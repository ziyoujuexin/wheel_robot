from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取功能包路径
    pkg_share = get_package_share_directory('people_follow')
    
    # People Follow节点
    people_follow_node = Node(
        package='people_follow',
        executable='people_follow',
        name='people_follow',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'people_follow_params.yaml')]
    )
    
    return LaunchDescription([
        people_follow_node,
    ])