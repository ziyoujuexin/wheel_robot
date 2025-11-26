from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    # param_file = os.path.join(
    #     get_package_share_directory('ft_driver'),
    #     'config',
    #     'params.yaml'
    # )
    param_file = '/home/wheeltec/wheeltec_ros2/src/FT/ft_driver/config/params.yaml'

    return LaunchDescription([
        Node(
            package='ft_driver',
            executable='subnode_ft',
            name='ft_node',
            output='screen',
            emulate_tty=True,
            parameters=[param_file],
            arguments=['--arg1'],
            condition=None
        ),
        Node(
            package='move_ft',
            executable='motorModel',
            name='moveNode',
            parameters=[param_file],
            output='screen'
        ),
    ])