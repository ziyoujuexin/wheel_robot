from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sound_track",
                executable="angle_publisher",
                name="sound_track_angle_publisher",
                parameters=[
                    {
                        "input_angle_topic": "",        # upstream UInt32 topic with real angle; leave empty to rely on fixed angle
                        "output_angle_topic": "awake_angle",
                        "publish_rate": 10.0,           # Hz
                        "use_fixed_angle": False,       # set True to use the fixed angle below
                        "angle": 0,                     # fallback fixed angle (degrees) if use_fixed_angle=True
                    }
                ],
                output="screen",
            )
        ]
    )
