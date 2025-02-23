from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcl_processor',
            executable='pcl_process',
            name='pcl_processor',
            output='screen',
            parameters=['config/params.yaml']
        )
    ]) 