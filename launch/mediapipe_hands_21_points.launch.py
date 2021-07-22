import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'mediapipe_hands_21_points.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('ros2_galactic_urdf_finger'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        #     arguments=[urdf]),
        Node(
            package='ros2_galactic_mediapipe_hands',
            executable='ros2_galactic_mediapipe_hands',
            name='ros2_galactic_mediapipe_hands',
            output='screen'),
        Node(
            package='image_tools',
            executable='cam2image',
            name='cam2image',
            output='screen'),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('ros2_galactic_urdf_finger'), 'mediapipe_hands_21_points.rviz')]
        )
        # Node(
        #     package='ros2_galactic_urdf_finger',
        #     executable='hand_21_points_state_publisher',
        #     name='hand_21_points_state_publisher',
        #     output='screen'),

    ])