from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safeplan_ros2',
            executable='safeplan_ros2_node',
            name='safeplan_ros2',
            output='screen',
            parameters=[{'yaml_path': 'config/algos.yaml'}]
        )
    ])