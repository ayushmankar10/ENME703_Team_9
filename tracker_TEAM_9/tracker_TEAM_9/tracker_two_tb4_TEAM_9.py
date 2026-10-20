#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import CompressedImage
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

from tracker_interfaces_team_9.msg import TrackerReport


def wrap_angle(angle):
    """Keep an angle between -pi and pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(q):
    """Extract yaw in radians."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def robot_id(namespace):
    """Derive an ID for TB4 names; other names need explicit IDs."""
    if namespace.startswith("tb") and namespace[2:].isdigit():
        value = int(namespace[2:]) - 400
        if 1 <= value <= 255:
            return value

    raise ValueError(
        f"Set an explicit my_id or opp_id for '{namespace}'"
    )


class TrackerTwoTB4(Node):
    def __init__(self):
        super().__init__("tracker_two_tb4")

        defaults = {
            "chaser": "tb402",
            "enemy": "tb407",
            "global_frame": "map",
            "my_id": -1,
            "opp_id": -1,
            "chaser_base_frame": "",
            "enemy_base_frame": "",
            "image_topic": "",
            "camera_topic": "",
            "nav_action_topic": "",
            "enemy_offset": 0.5,
            "enemy_fov": 80.0,
            "enemy_fov_offset": 30.0,
            "capture_cooldown": 5.0,
            "capture_distance_tolerance": 0.25,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.chaser = self.text_parameter("chaser").strip("/")
        self.enemy = self.text_parameter("enemy").strip("/")
        self.global_frame = self.text_parameter(
            "global_frame"
        ).strip("/")

        if (
            not self.chaser
            or not self.enemy
            or self.chaser == self.enemy
            or not self.global_frame
        ):
            raise ValueError(
                "Use different non-empty robot names and a global_frame"
            )

        my_id = int(self.get_parameter("my_id").value)
        opp_id = int(self.get_parameter("opp_id").value)

        self.my_id = (
            robot_id(self.chaser) if my_id == -1 else my_id
        )
        self.opp_id = (
            robot_id(self.enemy) if opp_id == -1 else opp_id
        )

        if not (
            0 <= self.my_id <= 255
            and 0 <= self.opp_id <= 255
            and self.my_id != self.opp_id
        ):
            raise ValueError(
                "Use different robot IDs between 0 and 255"
            )

        self.chaser_base = (
            self.text_parameter("chaser_base_frame").strip("/")
            or f"{self.chaser}/base_link"
        )
        self.enemy_base = (
            self.text_parameter("enemy_base_frame").strip("/")
            or f"{self.enemy}/base_link"
        )

        # image_topic matches tracker_sim.yaml.
        # camera_topic remains supported for existing launch files.
        image_topic = self.text_parameter("image_topic")
        camera_topic = self.text_parameter("camera_topic")

        if (
            image_topic
            and camera_topic
            and image_topic != camera_topic
        ):
            raise ValueError(
                "image_topic and camera_topic must agree if both are set"
            )

        self.image_topic = (
            image_topic
            or camera_topic
            or f"/{self.chaser}/oakd/rgb/preview/image_raw/compressed"
        )

        nav_action_topic = (
            self.text_parameter("nav_action_topic")
            or f"/{self.chaser}/navigate_to_pose"
        )

        self.enemy_offset = float(
            self.get_parameter("enemy_offset").value
        )
        fov = float(self.get_parameter("enemy_fov").value)
        buffer_angle = float(
            self.get_parameter("enemy_fov_offset").value
        )
        self.cooldown = float(
            self.get_parameter("capture_cooldown").value
        )
        self.distance_tolerance = float(
            self.get_parameter("capture_distance_tolerance").value
        )

        values = (
            self.enemy_offset,
            fov,
            buffer_angle,
            self.cooldown,
            self.distance_tolerance,
        )

        if not all(math.isfinite(value) for value in values):
            raise ValueError("Tracker parameters must be finite")

        self.exclusion_angle = math.radians(
            fov / 2.0 + buffer_angle
        )
        self.camera_half_fov = math.radians(fov / 2.0)

        if not (
            self.enemy_offset > 0.0
            and 0.0 < fov < 360.0
            and buffer_angle >= 0.0
            and self.exclusion_angle < math.pi
            and self.cooldown >= 5.0
            and 0.0 < self.distance_tolerance < self.enemy_offset
        ):
            raise ValueError(
                "Invalid tracker geometry or cooldown parameters"
            )

        self.images = deque(maxlen=30)

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
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
            nav_action_topic,
        )

        self.goal_handle = None
        self.pending_goal = False
        self.cancel_pending = False
        self.goal_succeeded = False
        self.tf_valid = False

        self.success_time = float("inf")
        self.last_goal = None
        self.last_send_time = float("-inf")
        self.last_report_time = float("-inf")

        self.alternative_side = 1.0
        self.use_alternative = False

        self.timer = self.create_timer(0.2, self.update_goal)

        self.get_logger().info(
            f"Tracking {self.enemy} (ID {self.opp_id}) "
            f"with {self.chaser} (ID {self.my_id}); "
            f"frames: {self.chaser_base}, {self.enemy_base}; "
            f"camera: {self.image_topic}"
        )

    def text_parameter(self, name):
        return str(self.get_parameter(name).value).strip()

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def tracker_image_callback(self, msg):
        self.images.append(msg)

    def world_pose(self, frame):
        """Return x, y and yaw in the shared map frame."""
        transform = self.tf_buffer.lookup_transform(
            self.global_frame,
            frame,
            Time(),
        )

        stamp = (
            Time.from_msg(transform.header.stamp).nanoseconds / 1e9
        )

        if stamp != 0.0:
            age = self.now_seconds() - stamp
            if not -0.1 <= age <= 1.0:
                raise ValueError(f"Stale TF for {frame}")

        position = transform.transform.translation
        pose = (
            position.x,
            position.y,
            quaternion_yaw(transform.transform.rotation),
        )

        if not all(math.isfinite(value) for value in pose):
            raise ValueError(f"Invalid TF for {frame}")

        return pose

    def desired_pose(self, enemy_pose, chaser_pose):
        """Choose a position in the opponent's operating zone."""
        ex, ey, enemy_yaw = enemy_pose
        cx, cy, _ = chaser_pose

        dx = cx - ex
        dy = cy - ey
        distance = math.hypot(dx, dy)
        angle = wrap_angle(math.atan2(dy, dx) - enemy_yaw)

        outside_fov = abs(angle) > self.exclusion_angle

        if (
            outside_fov
            and abs(distance - self.enemy_offset)
            <= self.distance_tolerance
        ):
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
        x, y, yaw = goal

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.global_frame
        transform.child_frame_id = f"{self.chaser}/tracker_goal"
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)

        self.tf_broadcaster.sendTransform(transform)

    def update_goal(self):
        try:
            enemy_pose = self.world_pose(self.enemy_base)
            chaser_pose = self.world_pose(self.chaser_base)
            self.tf_valid = True
        except (TransformException, ValueError) as error:
            self.tf_valid = False
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
        message = NavigateToPose.Goal()
        message.pose.header.frame_id = self.global_frame
        message.pose.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.pending_goal = True
        self.goal_succeeded = False
        self.last_send_time = self.now_seconds()

        try:
            future = self.nav_client.send_goal_async(message)
            future.add_done_callback(
                lambda done: self.goal_response_callback(done, goal)
            )
        except Exception as error:
            self.pending_goal = False
            self.get_logger().error(f"Could not send goal: {error}")

    def goal_response_callback(self, future, goal):
        self.pending_goal = False

        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self.get_logger().warning("Nav2 rejected the goal")
                return

            self.goal_handle = handle
            self.last_goal = goal

            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda done: self.goal_result_callback(done, handle)
            )

            if not self.tf_valid:
                self.cancel_current_goal()

        except Exception as error:
            self.get_logger().error(f"Goal response failed: {error}")

    def cancel_current_goal(self):
        if self.goal_handle is None or self.cancel_pending:
            return

        self.cancel_pending = True
        self.goal_succeeded = False
        handle = self.goal_handle

        try:
            future = handle.cancel_goal_async()
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
        if handle is not self.goal_handle:
            return

        self.goal_handle = None
        self.cancel_pending = False

        try:
            status = future.result().status
        except Exception as error:
            self.goal_succeeded = False
            self.get_logger().error(f"Goal result failed: {error}")
            return

        self.goal_succeeded = (
            status == GoalStatus.STATUS_SUCCEEDED and self.tf_valid
        )

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
        """Publish a geometrically eligible capture candidate.

        This does not verify visual identity or obstacle occlusion.
        Local report throttling is not master-server hit confirmation.
        """
        now = self.now_seconds()

        if not self.goal_succeeded:
            return

        if now - self.last_report_time < self.cooldown:
            return

        for image in reversed(self.images):
            stamp = Time.from_msg(image.header.stamp)
            image_time = stamp.nanoseconds / 1e9

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
            orientation = quaternion_yaw(
                enemy_tf.transform.rotation
            )

            if not all(
                math.isfinite(value)
                for value in (
                    distance, bearing, enemy_bearing, orientation
                )
            ):
                continue

            if (
                abs(distance - self.enemy_offset)
                > self.distance_tolerance
            ):
                continue
            if abs(enemy_bearing) <= self.exclusion_angle:
                continue
            if abs(bearing) > self.camera_half_fov:
                continue

            report = TrackerReport()
            report.my_id = self.my_id
            report.opp_id = self.opp_id
            report.image = image
            report.rel_dist = float(distance)
            report.rel_orient = float(math.degrees(orientation))

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
                try:
                    future = node.goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(
                        node, future, timeout_sec=1.0
                    )
                except Exception:
                    pass
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()