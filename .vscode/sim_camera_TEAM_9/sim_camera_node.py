#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""Publish labelled synthetic images for report pipeline testing only."""

import math

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from tf2_ros import Buffer, TransformException, TransformListener


class SimCamera(Node):
    def __init__(self):
        super().__init__("sim_camera")

        if not self.get_parameter("use_sim_time").value:
            raise RuntimeError(
                "Synthetic camera requires use_sim_time:=true."
            )

        defaults = {
            "chaser_base_frame": "tb3_0/base_footprint",
            "enemy_base_frame": "tb3_1/base_footprint",
            "image_topic": "/tb3_0/image_raw/compressed",
            "rate_hz": 10.0,
            "width": 320,
            "height": 240,
            "hfov_deg": 80.0,
            "max_range": 4.0,
            "robot_width": 0.2,
            "max_tf_age": 1.0,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def parameter(name):
            return self.get_parameter(name).value

        self.chaser_frame = str(parameter("chaser_base_frame"))
        self.enemy_frame = str(parameter("enemy_base_frame"))
        self.topic = str(parameter("image_topic"))
        self.width = int(parameter("width"))
        self.height = int(parameter("height"))
        self.rate = float(parameter("rate_hz"))
        self.hfov = math.radians(float(parameter("hfov_deg")))
        self.max_range = float(parameter("max_range"))
        self.robot_width = float(parameter("robot_width"))
        self.max_tf_age = float(parameter("max_tf_age"))

        numbers = (
            self.rate,
            self.hfov,
            self.max_range,
            self.robot_width,
            self.max_tf_age,
        )
        if not all(math.isfinite(value) and value > 0 for value in numbers):
            raise ValueError("Camera numeric parameters must be positive and finite.")
        if not 0 < self.hfov < math.pi:
            raise ValueError("hfov_deg must be between 0 and 180.")
        if not 64 <= self.width <= 1920 or not 64 <= self.height <= 1080:
            raise ValueError("Invalid image dimensions.")
        if not self.topic.startswith("/"):
            raise ValueError("image_topic must be an absolute topic.")

        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            CompressedImage,
            self.topic,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().warning(
            f"SYNTHETIC TEST images on {self.topic}. "
            "These are not camera photographs or valid capture evidence."
        )

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return

        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        horizon = self.height // 2
        image[:horizon] = (220, 205, 190)
        image[horizon:] = (80, 120, 80)
        label = "Opponent outside view"

        try:
            transform = self.buffer.lookup_transform(
                self.chaser_frame,
                self.enemy_frame,
                Time(),
            )
            stamp = Time.from_msg(
                transform.header.stamp,
                clock_type=now.clock_type,
            )
            age = (now - stamp).nanoseconds / 1e9

            if not -0.1 <= age <= self.max_tf_age:
                label = "TF stale: no opponent rendered"
            else:
                translation = transform.transform.translation
                forward = translation.x
                lateral = translation.y
                distance = math.hypot(forward, lateral)
                bearing = math.atan2(lateral, forward)

                visible = (
                    math.isfinite(distance)
                    and 0.05 < distance <= self.max_range
                    and forward > 0.05
                    and abs(bearing) <= self.hfov / 2.0
                )

                if visible:
                    focal = (self.width / 2.0) / math.tan(self.hfov / 2.0)
                    centre = self.width / 2.0 - focal * lateral / forward
                    half_width = focal * self.robot_width / (2.0 * forward)
                    body_height = min(
                        self.height,
                        max(4, int(half_width * 1.8)),
                    )

                    x0 = max(0, min(self.width - 1, int(centre - half_width)))
                    x1 = max(0, min(self.width - 1, int(centre + half_width)))
                    y1 = min(self.height - 1, horizon + body_height // 3)
                    y0 = max(0, y1 - body_height)

                    cv2.rectangle(image, (x0, y0), (x1, y1), (35, 35, 35), -1)
                    label = (
                        f"TF marker: {distance:.2f}m "
                        f"{math.degrees(bearing):+.0f}deg"
                    )

        except TransformException:
            label = "No relative TF"

        cv2.rectangle(image, (0, 0), (self.width - 1, 30), (0, 0, 170), -1)
        cv2.putText(
            image,
            "SYNTHETIC TEST",
            (6, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            image,
            label,
            (6, self.height - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1,
        )

        success, encoded = cv2.imencode(".jpg", image)
        if not success:
            return

        message = CompressedImage()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.chaser_frame
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimCamera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()