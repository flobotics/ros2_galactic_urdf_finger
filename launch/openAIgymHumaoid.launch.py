import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_prefix = LaunchConfiguration('xacro_prefix', default='')
    xacro_file_name = 'openAIgymHumanoid.urdf.xacro'
    xacro_file_path = os.path.join(
        get_package_share_directory('ros2_galactic_urdf_finger'),
        xacro_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'xacro_prefix',
            default_value='',
            description='Prefix to all joint frame names, to distinguish between different robots'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_l',
            namespace="humanoid",
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro',' ', xacro_file_path, ' prefix:=humanoid'])}]),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher_r',
        #     namespace="index",
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro',' ', xacro_file_path, ' prefix:=right'])}]),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('ros2_galactic_urdf_finger'), 'openAIgymHumanoid.rviz')]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace="humanoid",
            arguments=['robot_description:=/humanoid/robot_description'],
            output='screen'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            namespace="humanoid",
            output='screen',
            arguments = ["0", "0", "0.12", "0", "3.2", "0", "odom", "humanoid_openAIgymHumanoid_base"]),

    ])