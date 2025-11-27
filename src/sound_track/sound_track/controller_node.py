import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Int8, UInt32, String


class SoundTrackController(Node):
    """Basic sound tracking controller.

    Listens to /awake_angle and /sound_track_state. When state==1, it:
      1) Rotates toward the latest angle (0-180 turn right, 180-360 turn left).
      2) Drives forward until obstacle distance <= stop_range or timeout.
    """

    def __init__(self) -> None:
        super().__init__("sound_track_controller")

        self.declare_parameter("state_topic", "sound_track_state")
        self.declare_parameter("angle_topic", "awake_angle")
        self.declare_parameter("range_topic", "/tf_nova/range")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("angular_speed", 0.5)  # rad/s
        self.declare_parameter("linear_speed", 0.1)  # m/s
        self.declare_parameter("stop_range", 0.3)  # meters
        self.declare_parameter("forward_timeout", 5.0)  # seconds
        self.declare_parameter("control_rate", 20.0)  # Hz

        self.state_topic = self.get_parameter("state_topic").value
        self.angle_topic = self.get_parameter("angle_topic").value
        self.range_topic = self.get_parameter("range_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.stop_range = float(self.get_parameter("stop_range").value)
        self.forward_timeout = float(self.get_parameter("forward_timeout").value)
        self.control_rate = float(self.get_parameter("control_rate").value)

        self.last_angle: Optional[int] = None
        self.last_angle_time: Optional[rclpy.time.Time] = None
        self.last_range: Optional[float] = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Int8, self.state_topic, self._state_callback, 10)
        self.create_subscription(UInt32, self.angle_topic, self._angle_callback, 10)
        self.create_subscription(Range, self.range_topic, self._range_callback, 10)
        self.feedback_pub = self.create_publisher(String, "sound_track_feedback", 10)

        period = 1.0 / max(self.control_rate, 1.0)
        self.timer = self.create_timer(period, self._control_loop)

        self.phase = "idle"
        self.phase_end_time: Optional[rclpy.time.Time] = None
        self.current_twist = Twist()

        self.get_logger().info(
            f"[INIT] SoundTrackController ready. state_topic={self.state_topic}, "
            f"angle_topic={self.angle_topic}, range_topic={self.range_topic}, "
            f"cmd_vel_topic={self.cmd_vel_topic}"
        )

    def _angle_callback(self, msg: UInt32) -> None:
        self.last_angle = int(msg.data % 360)
        self.last_angle_time = self.get_clock().now()
        ts = self.last_angle_time.to_msg()
        self.get_logger().info(
            f"[ANGLE] angle={self.last_angle} ts={ts.sec}.{ts.nanosec:09d}"
        )

    def _range_callback(self, msg: Range) -> None:
        self.last_range = float(msg.range)
        self.get_logger().debug(f"[RANGE] {self.last_range:.3f} m")

    def _state_callback(self, msg: Int8) -> None:
        state = msg.data
        if state == 0:
            self.get_logger().info("[STATE] sound_track_state=0，进入待机并停转/停走")
            self._stop()
            return
        if state < 0:
            return
        if self.last_angle is None:
            self.get_logger().warn(f"[STATE] sound_track_state={state} but no angle available yet")
            return

        angle = self.last_angle
        angle_rad = 0.0
        angular_z = 0.0
        # 0~180: 左转，发布正的 z 角速度；180~360: 右转，发布负的 z 角速度
        if angle <= 180:
            angle_rad = math.radians(angle)
            angular_z = abs(self.angular_speed)
        else:
            angle_rad = math.radians(360 - angle)
            angular_z = -abs(self.angular_speed)

        duration = angle_rad / max(abs(self.angular_speed), 1e-3)
        self.phase = "rotate"
        self.phase_end_time = self.get_clock().now() + rclpy.time.Duration(
            seconds=duration
        )
        self.current_twist = Twist()
        self.current_twist.angular.z = angular_z

        angle_ts = (
            f"{self.last_angle_time.to_msg().sec}.{self.last_angle_time.to_msg().nanosec:09d}"
            if self.last_angle_time
            else "unknown"
        )
        self.get_logger().info(
            f"[ROTATE] state={state}, angle={angle} deg (ts {angle_ts}), "
            f"angular_z={angular_z:.2f}, duration={duration:.2f}s"
        )

    def _control_loop(self) -> None:
        now = self.get_clock().now()

        if self.phase == "rotate":
            if self.phase_end_time and now >= self.phase_end_time:
                # switch to forward
                self.phase = "forward"
                self.phase_end_time = now + rclpy.time.Duration(
                    seconds=self.forward_timeout
                )
                self.current_twist = Twist()
                self.current_twist.linear.x = self.linear_speed
                self.get_logger().info(
                    f"[FORWARD] Rotation done, start forward linear.x={self.linear_speed:.2f}, "
                    f"timeout={self.forward_timeout}s"
                )
            else:
                self.cmd_pub.publish(self.current_twist)
                return

        if self.phase == "forward":
            # Stop if range too close or timeout reached
            if (self.last_range is not None and self.last_range <= self.stop_range) or (
                self.phase_end_time and now >= self.phase_end_time
            ):
                reason = "timeout" if self.phase_end_time and now >= self.phase_end_time else "obstacle"
                self.get_logger().info(
                    f"[STOP] Forward stop: range={self.last_range}, reason={reason}"
                )
                self._stop(reason)
            else:
                self.cmd_pub.publish(self.current_twist)

    def _stop(self, reason: str = "manual") -> None:
        self.current_twist = Twist()
        self.cmd_pub.publish(self.current_twist)
        self.phase = "idle"
        self.phase_end_time = None
        feedback = String()
        feedback.data = f"sound_track_stopped:{reason}"
        self.feedback_pub.publish(feedback)
        self.get_logger().info(f"[STOP] phase=idle, reason={reason}, last_range={self.last_range}")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SoundTrackController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
