import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_prefix = LaunchConfiguration('xacro_prefix', default='')
    xacro_file_name = 'one_finger.urdf.xacro'
    xacro_file_path = os.path.join(
        get_package_share_directory('ros2_galactic_urdf_finger'),
        xacro_file_name)

    # urdf_file_name = 'angle_finger_test.urdf.xml'
    # urdf = os.path.join(
    #     get_package_share_directory('ros2_galactic_urdf_finger'),
    #     urdf_file_name)
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()

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
            name='robot_state_publisher',
            namespace="index",
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro',' ', xacro_file_path, ' prefix:=index'])}]),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace="middle",
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro',' ', xacro_file_path, ' prefix:=middle']) }] ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace="ring",
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro',' ', xacro_file_path, ' prefix:=ring']) }] ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace="pinky",
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro',' ', xacro_file_path, ' prefix:=pinky']) }] ),
        # Node(
        #     package='ros2_galactic_mediapipe_hands',
        #     executable='ros2_galactic_mediapipe_hands_angle',
        #     name='ros2_galactic_mediapipe_hands_angle',
        #     output='screen'),
        # Node(
        #     package='image_tools',
        #     executable='cam2image',
        #     name='cam2image',
        #     output='screen'),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('ros2_galactic_urdf_finger'), 'hand.rviz')]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace="index",
            arguments=['robot_description:=/index/robot_description'],
            output='screen'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui2',
            namespace="middle",
            arguments=['robot_description:=/middle/robot_description'],
            output='screen'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui2',
            namespace="ring",
            arguments=['robot_description:=/ring/robot_description'],
            output='screen'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui2',
            namespace="pinky",
            arguments=['robot_description:=/pinky/robot_description'],
            output='screen'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            namespace="index",
            output='screen',
            arguments = ["0", "0", "0", "0", "0", "0", "odom", "index_metacarpals"]),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher2',
            namespace="middle",
            output='screen',
            arguments = ["0", "0.02", "0", "0", "0", "0", "odom", "middle_metacarpals"]),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher2',
            namespace="ring",
            output='screen',
            arguments = ["0", "0.04", "0", "0", "0", "0", "odom", "ring_metacarpals"]),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher2',
            namespace="pinky",
            output='screen',
            arguments = ["0", "0.06", "0", "0", "0", "0", "odom", "pinky_metacarpals"]),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui2',
        #     namespace='test',
        #     output='screen'),

    ])