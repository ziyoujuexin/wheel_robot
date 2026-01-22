import re
from typing import Optional, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32, String

try:
    import serial  # type: ignore
except ImportError:
    serial = None


def parse_ports(port_list: str) -> List[str]:
    return [p.strip() for p in port_list.split(",") if p.strip()]


class SerialAngleNode(Node):
    """Read sound angle logs from a serial port and publish /awake_angle (UInt32)."""

    def __init__(self) -> None:
        super().__init__("serial_angle_node")
        self.declare_parameter(
            "ports",
            "/dev/ttyCH343USB1,/dev/ttyCH343USB0,/dev/ttyCH343USB2,/dev/ttyCH341USB0,/dev/ttyCH341USB1,/dev/ttyCH341USB2,/dev/ttyAMA0",
        )
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("regex_pattern", r"Detected Angle:\s*(\d+)")
        self.declare_parameter("output_topic", "awake_angle")
        self.declare_parameter("debug_topic", "serial_data_debug")
        self.declare_parameter("poll_ms", 100)

        ports_param = self.get_parameter("ports").get_parameter_value().string_value
        self.ports = parse_ports(ports_param)
        self.baud = int(self.get_parameter("baud_rate").get_parameter_value().integer_value)
        self.pattern = re.compile(self.get_parameter("regex_pattern").get_parameter_value().string_value)
        self.poll_ms = int(self.get_parameter("poll_ms").get_parameter_value().integer_value)
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        debug_topic = self.get_parameter("debug_topic").get_parameter_value().string_value

        self.pub_angle = self.create_publisher(UInt32, output_topic, 10)
        self.pub_debug = self.create_publisher(String, debug_topic, 10)
        self.last_angle: Optional[int] = None
        self.last_angle_time = None

        self.serial_port: Optional[serial.Serial] = None
        self.current_port_index = 0
        if serial is None:
            self.get_logger().error("python3-serial not installed; cannot read serial ports.")
        else:
            self._open_first_available()

        period = max(self.poll_ms, 10) / 1000.0
        self.timer = self.create_timer(period, self._poll)

    def _open_first_available(self) -> None:
        for idx, port in enumerate(self.ports):
            try:
                self.get_logger().info(f"尝试打开串口: {port}")
                sp = serial.Serial(port=port, baudrate=self.baud, timeout=0.1)
                if sp.is_open:
                    self.serial_port = sp
                    self.current_port_index = idx
                    self.get_logger().info(f"串口打开成功: {port}")
                    return
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"无法打开 {port}: {e}")
        self.get_logger().error("未能打开任何串口，节点将持续重试但当前不会发布角度")

    def _poll(self) -> None:
        if serial is None:
            return
        if self.serial_port is None or not getattr(self.serial_port, "is_open", False):
            self._switch_port()
            return
        try:
            available = self.serial_port.in_waiting
            if available <= 0:
                return
            data = self.serial_port.read(available).decode(errors="ignore")
            if data:
                dbg = String()
                dbg.data = data
                self.pub_debug.publish(dbg)
                angle = self._parse_angle(data)
                if angle is not None:
                    msg = UInt32()
                    msg.data = angle % 360
                    self.pub_angle.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"串口读写错误: {e}")
            self._switch_port()

    def _parse_angle(self, data: str) -> Optional[int]:
        match = self.pattern.search(data)
        if match:
            try:
                angle = int(match.group(1))
                self.last_angle = angle % 360
                self.last_angle_time = self.get_clock().now()
                return self.last_angle
            except ValueError:
                self.get_logger().warn("角度转换失败")
        else:
            self._switch_port()
        return None

    def _switch_port(self) -> None:
        if not self.ports:
            return
        try:
            if self.serial_port:
                try:
                    current = self.serial_port.port  # type: ignore[attr-defined]
                except Exception:
                    current = "unknown"
                self.serial_port.close()
                self.get_logger().warn(f"切换串口，当前关闭: {current}")
        except Exception:
            pass

        self.current_port_index = (self.current_port_index + 1) % len(self.ports)
        next_port = self.ports[self.current_port_index]
        try:
            self.get_logger().warn(f"尝试切换到串口: {next_port}")
            sp = serial.Serial(port=next_port, baudrate=self.baud, timeout=0.1)
            if sp.is_open:
                self.serial_port = sp
                self.get_logger().info(f"串口切换成功: {next_port}")
                return
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"无法打开 {next_port}: {e}")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SerialAngleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
