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
            ),
            Node(
                package="sound_track",
                executable="sound_track_controller",
                name="sound_track_controller",
                parameters=[
                    {
                        "state_topic": "sound_track_state",
                        "angle_topic": "awake_angle",
                        "range_topic": "/tf_nova/range",
                        "cmd_vel_topic": "cmd_vel",
                        "angular_speed": 0.5,
                        "linear_speed": 0.1,
                        "stop_range": 0.3,
                        "forward_timeout": 5.0,
                        "control_rate": 20.0,
                    }
                ],
                output="screen",
            )
        ]
    )
