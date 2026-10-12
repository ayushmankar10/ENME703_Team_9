#!/usr/bin/env python3

import math
from collections import deque

import rclpy  # pyright: ignore[reportMissingImports]
from rclpy.action import ActionClient  # pyright: ignore[reportMissingImports]
from rclpy.node import Node  # pyright: ignore[reportMissingImports]
from rclpy.qos import qos_profile_sensor_data  # pyright: ignore[reportMissingImports]
from rclpy.time import Time  # pyright: ignore[reportMissingImports]
from rclpy.executors import ExternalShutdownException  # pyright: ignore[reportMissingImports]

from action_msgs.msg import GoalStatus  # pyright: ignore[reportMissingImports]
from geometry_msgs.msg import TransformStamped  # pyright: ignore[reportMissingImports]
from nav2_msgs.action import NavigateToPose  # pyright: ignore[reportMissingImports]
from sensor_msgs.msg import CompressedImage  # pyright: ignore[reportMissingImports]
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException  # pyright: ignore[reportMissingImports]

from tracker_interfaces_team_9.msg import TrackerReport  # pyright: ignore[reportMissingImports]


def wrap_angle(angle):
    """Keep an angle between -pi and pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(q):
    """Extract yaw in radians from a quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def robot_id(namespace):
    """Convert tb402 to ID 2, tb407 to ID 7, etc."""
    if not namespace.startswith("tb") or not namespace[2:].isdigit():
        raise ValueError("Robot names must look like tb402 or tb407")

    value = int(namespace[2:]) - 400

    if not 1 <= value <= 255:
        raise ValueError("Robot ID must fit uint8: tb401 maps to ID 1")

    return value


class TrackerTwoTB4(Node):
    def __init__(self):
        super().__init__("tracker_two_tb4")

        defaults = {
            "chaser": "tb402",
            "enemy": "tb407",
            "global_frame": "map",
            "enemy_offset": 0.5,
            "enemy_fov": 80.0,
            "enemy_fov_offset": 30.0,
            "capture_cooldown": 5.0,
            "capture_distance_tolerance": 0.25,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.chaser = str(
            self.get_parameter("chaser").value
        ).strip("/")

        self.enemy = str(
            self.get_parameter("enemy").value
        ).strip("/")

        self.global_frame = str(
            self.get_parameter("global_frame").value
        ).strip("/")

        self.my_id = robot_id(self.chaser)
        self.opp_id = robot_id(self.enemy)

        if self.chaser == self.enemy or not self.global_frame:
            raise ValueError(
                "Use different robots and a non-empty global_frame"
            )

        self.enemy_offset = float(
            self.get_parameter("enemy_offset").value
        )

        fov = float(self.get_parameter("enemy_fov").value)
        buffer = float(self.get_parameter("enemy_fov_offset").value)

        # Excluded region is centred on the opponent's forward direction.
        self.exclusion_angle = math.radians(fov / 2.0 + buffer)
        self.camera_half_fov = math.radians(fov / 2.0)

        self.cooldown = float(
            self.get_parameter("capture_cooldown").value
        )

        self.distance_tolerance = float(
            self.get_parameter("capture_distance_tolerance").value
        )

        if not (
            self.enemy_offset > 0.0
            and 0.0 < fov < 360.0
            and buffer >= 0.0
            and self.exclusion_angle < math.pi
            and self.cooldown >= 5.0
            and 0.0 < self.distance_tolerance < self.enemy_offset
        ):
            raise ValueError(
                "Invalid tracker geometry or cooldown parameters"
            )

        self.chaser_base = f"{self.chaser}/base_link"
        self.enemy_base = f"{self.enemy}/base_link"

        # Keep recent images so TF can catch up to their timestamps.
        self.images = deque(maxlen=30)

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.image_sub = self.create_subscription(
            CompressedImage,
            f"/{self.chaser}/oakd/rgb/preview/image_raw/compressed",
            self.tracker_image_callback,
            qos_profile_sensor_data,
        )

        self.report_pub = self.create_publisher(
            TrackerReport,
            f"/{self.chaser}/tracker_report",
            10,
        )

        self.report_image_pub = self.create_publisher(
            CompressedImage,
            f"/{self.chaser}/tracker_report_img/compressed",
            10,
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            f"/{self.chaser}/navigate_to_pose",
        )

        self.goal_handle = None
        self.pending_goal = False
        self.cancel_pending = False
        self.goal_succeeded = False

        self.success_time = float("inf")
        self.last_goal = None
        self.last_send_time = float("-inf")
        self.last_report_time = float("-inf")

        self.alternative_side = 1.0
        self.use_alternative = False

        self.timer = self.create_timer(0.2, self.update_goal)  # 5 Hz

        self.get_logger().info(
            f"Tracking {self.enemy} with {self.chaser}"
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def tracker_image_callback(self, msg):
        self.images.append(msg)

    def world_pose(self, frame):
        """Return a robot's x, y and yaw in the global frame."""
        tf = self.tf_buffer.lookup_transform(
            self.global_frame,
            frame,
            Time(),
        )

        stamp = Time.from_msg(tf.header.stamp).nanoseconds / 1e9

        if stamp != 0.0:
            age = self.now_seconds() - stamp

            if not -0.1 <= age <= 1.0:
                raise ValueError(f"Stale TF for {frame}")

        position = tf.transform.translation

        pose = (
            position.x,
            position.y,
            quaternion_yaw(tf.transform.rotation),
        )

        if not all(math.isfinite(value) for value in pose):
            raise ValueError(f"Invalid TF for {frame}")

        return pose

    def desired_pose(self, enemy_pose, chaser_pose):
        """Choose an allowed position facing the opponent."""
        ex, ey, enemy_yaw = enemy_pose
        cx, cy, _ = chaser_pose

        dx = cx - ex
        dy = cy - ey

        distance = math.hypot(dx, dy)
        angle = wrap_angle(
            math.atan2(dy, dx) - enemy_yaw
        )

        outside_fov = abs(angle) > self.exclusion_angle

        if (
            outside_fov
            and abs(distance - self.enemy_offset)
            <= self.distance_tolerance
        ):
            # At capture distance: hold position and face the opponent.
            return cx, cy, math.atan2(ey - cy, ex - cx)

        if self.use_alternative:
            angle = self.alternative_side * min(
                math.pi,
                self.exclusion_angle + math.radians(15.0),
            )

        elif not outside_fov:
            side = 1.0 if angle >= 0.0 else -1.0

            angle = side * min(
                math.pi,
                self.exclusion_angle + math.radians(10.0),
            )

        gx = ex + self.enemy_offset * math.cos(enemy_yaw + angle)
        gy = ey + self.enemy_offset * math.sin(enemy_yaw + angle)

        goal_yaw = math.atan2(ey - gy, ex - gx)

        return gx, gy, goal_yaw

    def broadcast_goal(self, goal):
        """Publish the desired pose as a TF frame for inspection."""
        x, y, yaw = goal

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.global_frame
        tf.child_frame_id = f"{self.chaser}/tracker_goal"

        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)

        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)

        self.tf_broadcaster.sendTransform(tf)

    def update_goal(self):
        try:
            enemy_pose = self.world_pose(self.enemy_base)
            chaser_pose = self.world_pose(self.chaser_base)

        except (TransformException, ValueError) as error:
            self.goal_succeeded = False
            self.cancel_current_goal()

            self.get_logger().warning(
                f"Waiting for valid TF: {error}",
                throttle_duration_sec=2.0,
            )
            return

        goal = self.desired_pose(enemy_pose, chaser_pose)

        self.broadcast_goal(goal)
        self.publish_report_if_ready()

        if self.pending_goal or self.cancel_pending:
            return

        # Limit goal changes to at most once per second.
        if self.now_seconds() - self.last_send_time < 1.0:
            return

        changed = self.last_goal is None

        if self.last_goal is not None:
            lx, ly, lyaw = self.last_goal

            changed = (
                math.hypot(goal[0] - lx, goal[1] - ly) > 0.15
                or abs(wrap_angle(goal[2] - lyaw)) > 0.2
            )

        if self.goal_handle is not None:
            if changed:
                self.cancel_current_goal()
            return

        if changed or not self.goal_succeeded:
            self.send_goal(goal)

    def send_goal(self, goal):
        if not self.nav_client.server_is_ready():
            self.get_logger().warning(
                "Waiting for Nav2 action server",
                throttle_duration_sec=2.0,
            )
            return

        x, y, yaw = goal

        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = self.global_frame
        msg.pose.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)

        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.pending_goal = True
        self.goal_succeeded = False
        self.last_send_time = self.now_seconds()

        try:
            future = self.nav_client.send_goal_async(msg)

            future.add_done_callback(
                lambda result: self.goal_response_callback(result, goal)
            )

        except Exception as error:
            self.pending_goal = False
            self.get_logger().error(
                f"Could not send goal: {error}"
            )

    def goal_response_callback(self, future, goal):
        self.pending_goal = False

        try:
            handle = future.result()

            if handle is None or not handle.accepted:
                self.get_logger().warning("Nav2 rejected the goal")
                return

            self.goal_handle = handle
            self.last_goal = goal

            result = handle.get_result_async()

            result.add_done_callback(
                lambda done: self.goal_result_callback(done, handle)
            )

        except Exception as error:
            self.get_logger().error(
                f"Goal response failed: {error}"
            )

    def cancel_current_goal(self):
        if self.goal_handle is None or self.cancel_pending:
            return

        self.cancel_pending = True
        self.goal_succeeded = False

        try:
            future = self.goal_handle.cancel_goal_async()
            handle = self.goal_handle

            future.add_done_callback(
                lambda done: self.cancel_response_callback(done, handle)
            )

        except Exception as error:
            self.cancel_pending = False
            self.get_logger().error(
                f"Cancellation request failed: {error}"
            )

    def cancel_response_callback(self, future, handle):
        if handle is not self.goal_handle:
            return

        try:
            response = future.result()

            if response is None or not response.goals_canceling:
                self.cancel_pending = False

                self.get_logger().warning(
                    "Cancellation not accepted; waiting for goal result"
                )

        except Exception as error:
            self.cancel_pending = False
            self.get_logger().error(
                f"Cancellation response failed: {error}"
            )

    def goal_result_callback(self, future, handle):
        # Ignore a result belonging to an older goal.
        if handle is not self.goal_handle:
            return

        self.goal_handle = None
        self.cancel_pending = False

        try:
            result = future.result()
            status = result.status

        except Exception as error:
            self.goal_succeeded = False
            self.get_logger().error(
                f"Goal result failed: {error}"
            )
            return

        self.goal_succeeded = status == GoalStatus.STATUS_SUCCEEDED

        if self.goal_succeeded:
            self.success_time = self.now_seconds()
            self.use_alternative = False

            self.get_logger().info(
                "Goal reached; waiting for a valid capture image"
            )

        elif status == GoalStatus.STATUS_ABORTED:
            self.alternative_side *= -1.0
            self.use_alternative = True

            self.get_logger().warning(
                "Goal aborted; trying another allowed position"
            )

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(
                "Old goal cancelled; updating target"
            )

        else:
            self.get_logger().warning(
                f"Goal finished with status {status}"
            )

    def publish_report_if_ready(self):
        now = self.now_seconds()

        if not self.goal_succeeded:
            return

        if now - self.last_report_time < self.cooldown:
            return

        for image in reversed(self.images):
            stamp = Time.from_msg(image.header.stamp)
            image_time = stamp.nanoseconds / 1e9

            # Use a fresh image taken after the navigation goal succeeded.
            if image_time <= 0.0 or image_time < self.success_time:
                continue

            if not 0.0 <= now - image_time <= 1.0:
                continue

            if not image.data:
                continue

            try:
                enemy_tf = self.tf_buffer.lookup_transform(
                    self.chaser_base,
                    self.enemy_base,
                    stamp,
                )

                chaser_tf = self.tf_buffer.lookup_transform(
                    self.enemy_base,
                    self.chaser_base,
                    stamp,
                )

            except TransformException:
                continue

            relative = enemy_tf.transform.translation
            reverse = chaser_tf.transform.translation

            distance = math.hypot(relative.x, relative.y)
            bearing = math.atan2(relative.y, relative.x)
            enemy_bearing = math.atan2(reverse.y, reverse.x)

            if (
                abs(distance - self.enemy_offset)
                > self.distance_tolerance
            ):
                continue

            # Reject capture from inside the opponent's excluded region.
            if abs(enemy_bearing) <= self.exclusion_angle:
                continue

            # The opponent must be in front of the chaser's camera.
            if abs(bearing) > self.camera_half_fov:
                continue

            report = TrackerReport()

            report.header.stamp = image.header.stamp
            report.header.frame_id = self.chaser_base
            report.image = image

            report.my_id = self.my_id
            report.opp_id = self.opp_id
            report.rel_dist = float(distance)

            report.rel_orient = float(
                math.degrees(
                    quaternion_yaw(enemy_tf.transform.rotation)
                )
            )

            self.report_pub.publish(report)
            self.report_image_pub.publish(image)

            self.last_report_time = now

            self.get_logger().info(
                f"Published report: opponent {self.opp_id}, "
                f"distance {distance:.2f} m"
            )
            return


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TrackerTwoTB4()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            if rclpy.ok() and node.goal_handle is not None:
                future = node.goal_handle.cancel_goal_async()

                rclpy.spin_until_future_complete(
                    node,
                    future,
                    timeout_sec=1.0,
                )

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()