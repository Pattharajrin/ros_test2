from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # รัน RPLidar Node และ remap /scan -> /rplidar_scan
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar',
            output='screen',
            parameters=[{'use_sim_time': False}],
            remappings=[('/scan', '/rplidar_scan')]
        ),
        # รัน Lidar Reader Node
        Node(
            package='turtlebot3_lidar_control',
            executable='lidar_reader',
            name='lidar_reader',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        # รัน TurtleBot Controller Node
        Node(
            package='turtlebot3_lidar_control',
            executable='turtlebot_controller',
            name='turtlebot_controller',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        # รัน Obstacle Detector Node
        Node(
            package='turtlebot3_lidar_control',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])
