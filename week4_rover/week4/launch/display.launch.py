import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    urdf = os.path.join(
    get_package_share_directory('week4'),
    'urdf',
    'my_rover.urdf')
    
    rviz_config = os.path.join(
    get_package_share_directory('week4_arm'),
    'rviz',
    'rover.rviz')
    


    
    # Read the URDF file
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Check if RViz config exists
    rviz_args = []
    if os.path.exists(rviz_config):
        rviz_args = ['-d', rviz_config]
        print(f"Using RViz config: {rviz_config}")
    else:
        print(f"RViz config not found at: {rviz_config}")
        print("RViz will start with default configuration")

    return LaunchDescription([
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        
        # RViz with config (if exists)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=rviz_args,
        ),
    ])
