import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    urdf = os.path.join(
    get_package_share_directory('week4'),
    'urdf',
    'my_rover_gazebo.urdf')
    
    # Get the urdf file path
    controller_config = os.path.join(
    get_package_share_directory('week4'),
    'config',
    'rover_controllers.yaml')
    
    rviz_config = os.path.join(
    get_package_share_directory('week4'),
    'rviz',
    'rover.rviz')
    

    
    # Get the controller config file path
    controller_config = '/home/saksham-22/kratos/src/week4/config/rover_controllers.yaml'
    
    # Read the URDF file
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'),
                '/launch/gazebo.launch.py'
            ]),
            launch_arguments={'verbose': 'false'}.items()
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'my_rover', '-topic', 'robot_description'],
        ),

        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[controller_config],
        ),

        # Load and start controllers
        Node(
            package='controller_manager',
            executable='spawner',
            name='joint_state_broadcaster_spawner',
            output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),

        # Steering controllers
        Node(
            package='controller_manager',
            executable='spawner',
            name='front_left_steering_controller_spawner',
            output='screen',
            arguments=['front_left_steering_controller', '--controller-manager', '/controller_manager'],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            name='front_right_steering_controller_spawner',
            output='screen',
            arguments=['front_right_steering_controller', '--controller-manager', '/controller_manager'],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            name='rear_left_steering_controller_spawner',
            output='screen',
            arguments=['rear_left_steering_controller', '--controller-manager', '/controller_manager'],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            name='rear_right_steering_controller_spawner',
            output='screen',
            arguments=['rear_right_steering_controller', '--controller-manager', '/controller_manager'],
        ),

        # Drive controllers
        Node(
            package='controller_manager',
            executable='spawner',
            name='front_left_drive_controller_spawner',
            output='screen',
            arguments=['front_left_drive_controller', '--controller-manager', '/controller_manager'],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            name='front_right_drive_controller_spawner',
            output='screen',
            arguments=['front_right_drive_controller', '--controller-manager', '/controller_manager'],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            name='rear_left_drive_controller_spawner',
            output='screen',
            arguments=['rear_left_drive_controller', '--controller-manager', '/controller_manager'],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            name='rear_right_drive_controller_spawner',
            output='screen',
            arguments=['rear_right_drive_controller', '--controller-manager', '/controller_manager'],
        ),

        # RViz (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
