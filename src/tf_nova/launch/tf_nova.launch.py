from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tf_nova",
                executable="tf_nova_node",
                name="tf_nova",
                parameters=[
                    {
                        "port": "/dev/ttyTHS1",
                        "baud_rate": 115200,
                        "frame_header": 0x59,
                        "use_mm_format": False,
                        "frame_id": "tf_nova_link",
                        "min_range": 0.1,
                        "max_range": 7.0,
                        "confidence_threshold": 0,
                        "range_topic": "tf_nova/range",
                    }
                ],
                output="screen",
            )
        ]
    )
