from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    tf_nova_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("tf_nova"), "/launch/tf_nova.launch.py"]
        )
    )

    return LaunchDescription(
        [
            tf_nova_launch,
            Node(
                package="sound_track",
                executable="serial_angle_node",
                name="serial_angle_node",
                parameters=[
                    {
                        "ports": "/dev/ttyCH343USB1,/dev/ttyCH343USB0,/dev/ttyCH343USB2",
                        "baud_rate": 115200,
                        "regex_pattern": r"Detected Angle:\s*(\d+)",
                        "output_topic": "awake_angle",
                        "debug_topic": "serial_data_debug",
                        "poll_ms": 100,
                    }
                ],
                output="screen",
            )
        ]
    )
