from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32


class AnglePublisher(Node):
    """Relays real awake angles to a target topic, optionally with a fixed fallback."""

    def __init__(self) -> None:
        super().__init__("sound_track_angle_publisher")
        self.declare_parameter("input_angle_topic", "")
        self.declare_parameter("output_angle_topic", "awake_angle")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("use_fixed_angle", False)
        self.declare_parameter("angle", 0)

        input_topic = self.get_parameter("input_angle_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_angle_topic").get_parameter_value().string_value
        self.publish_rate = float(self.get_parameter("publish_rate").get_parameter_value().double_value)
        self.use_fixed_angle = bool(self.get_parameter("use_fixed_angle").get_parameter_value().bool_value)
        self.current_angle: Optional[int] = None

        if self.use_fixed_angle:
            self.current_angle = int(self.get_parameter("angle").get_parameter_value().integer_value)

        self.pub = self.create_publisher(UInt32, output_topic, 10)

        if input_topic:
            if input_topic == output_topic:
                self.get_logger().warn(
                    f"input_angle_topic equals output_angle_topic ({input_topic}); relay disabled to avoid loop"
                )
                self.sub = None
            else:
                self.sub = self.create_subscription(UInt32, input_topic, self._angle_callback, 10)
        else:
            self.sub = None

        period = 1.0 / max(self.publish_rate, 0.01)
        self.timer = self.create_timer(period, self._publish_angle)
        self.get_logger().info(
            f"sound_track angle relay started: output={output_topic}, "
            f"input={'none' if not input_topic else input_topic}, "
            f"use_fixed_angle={self.use_fixed_angle}, "
            f"publish_rate={self.publish_rate} Hz"
        )

    def _angle_callback(self, msg: UInt32) -> None:
        self.current_angle = int(msg.data) % 360

    def _publish_angle(self) -> None:
        if self.current_angle is None:
            # No data received and no fixed angle; skip publish
            return
        msg = UInt32()
        msg.data = self.current_angle
        self.pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = AnglePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
