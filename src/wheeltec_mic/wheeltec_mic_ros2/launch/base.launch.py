import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ========== 新增：设置环境变量，解决共享内存冲突 ==========
    set_fastdds_env = SetEnvironmentVariable(
        name='FASTDDS_BUILTIN_TRANSPORTS',
        value='UDPv4'
    )
    
    mic_dir = get_package_share_directory('wheeltec_mic_ros2')
    mic_launch_dir = os.path.join(mic_dir, 'launch')
    mic_include_dir = os.path.join(mic_launch_dir, 'include')
    command_config = os.path.join(mic_dir, 'config', 'param.yaml')
    audio_path = os.path.join(mic_dir,'feedback_voice')
    resource_param = {"audio_path": audio_path}
    print(audio_path)

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # 获取bodyreader包的路径
    # bodyreader_dir = get_package_share_directory('bodyreader')
    
    # 获取ft_driver包的路径（假设base_ft.launch.py在ft_driver包中）
    ft_driver_dir = get_package_share_directory('ft_driver')
    ft_launch_dir = os.path.join(ft_driver_dir, 'launch')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
    )

    # 注释掉导航配置（已关闭）
    # wheeltec_nav = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(mic_include_dir, 'voi_navigation.launch.py')),
    # )

      
    # body_follow = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(bodyreader_dir, 'launch', 'bodyfollow.launch.py')),
    # )
    
    # 添加base_ft.launch.py的启动
    base_ft_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ft_launch_dir, 'base_ft.launch.py')),
    )

    # call_recognition = Node(
    #     package="wheeltec_mic_ros2",
    #     executable="call_recognition",
    #     #output='screen',
    #     parameters=[command_config]
    # )
    

    # command_recognition = Node(
    #     package="wheeltec_mic_ros2",
    #     executable="command_recognition",
    #     output='screen',
    #     parameters=[resource_param,
    #         command_config]                     
    # )
    
    node_feedback = Node(
        package="wheeltec_mic_ros2",
        executable="node_feedback",
        output='screen',
        parameters=[resource_param]
    )

    motion_control = Node(
        package="wheeltec_mic_ros2",
        executable="motion_control",
        output='screen',
        parameters=[resource_param,
        command_config]   
    )

    lasertracker = Node(
        package="simple_follower_ros2", 
        executable="lasertracker", 
        name='lasertracker'
    )

    serial_angle_node = Node(
    package="serial_angle_pkg",
    executable="serial_angle_node",
    name='serial_angle_node',
    output='screen'  # 可选，用于在终端显示输出
    )

    ld = LaunchDescription()
    
    # ========== 新增：首先添加环境变量设置 ==========
    # 这个必须在所有节点之前添加，确保环境变量先被设置
    ld.add_action(set_fastdds_env)
    
    # 然后按顺序添加你的所有节点
    ld.add_action(serial_angle_node)
    ld.add_action(wheeltec_robot)
    ld.add_action(wheeltec_lidar)
    # ld.add_action(wheeltec_nav)  # 导航配置已关闭
    ld.add_action(lasertracker)

    # ld.add_action(body_follow)    

    ld.add_action(base_ft_launch)  # 添加base_ft启动

  
    # ld.add_action(call_recognition)
    # ld.add_action(command_recognition)
    
    ld.add_action(node_feedback)
    ld.add_action(motion_control)
    
    return ld