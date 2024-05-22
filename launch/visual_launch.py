import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('pose_estimation'), 'config', 'ulite6_pepper.rviz')],
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true'
        }.items()
    )

    ulite6_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('xarm_api'),'launch', 'lite6_driver.launch.py')
            ),
            launch_arguments={'robot_ip': '192.168.1.168'}.items()
            )
    
    xarm_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pose_estimation'), 'launch', 'lite6_control_rviz_display.launch.py')
        ),
        launch_arguments={
            'robot_ip': '192.168.1.168',
            'add_gripper': 'true'
        }.items()
    )
    # xarm_rviz = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('xarm_controller'), 'launch', 'lite6_control_rviz_display.launch.py')
    #     ),
    #     launch_arguments={
    #         'robot_ip': '192.168.1.168',
    #         'add_gripper': 'true'
    #     }.items()
    # )

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0.065', '0', '0.003', '1.5', '0', '0', 'link6', 'camera_color_optical_frame'],
        output='screen'
    )

    static_transform_publisher_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_2',
        arguments=['0', '0', '0', '0', '0', '0', 'frame_id', 'child_frame_id'],
        output='screen'
    )

    return LaunchDescription([
        realsense_node,
        TimerAction(period=1.0, actions=[ulite6_bringup]),
        TimerAction(period=4.0, actions=[static_transform_publisher]),
        TimerAction(period=2.0, actions=[xarm_rviz]),
        TimerAction(period=3.0, actions=[rviz_node]),
        TimerAction(period=6.0, actions=[static_transform_publisher_2])
    ])
