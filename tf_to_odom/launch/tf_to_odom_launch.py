from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf_to_odom',
            executable='tf_to_odom_node',
            name='tf_to_odom_node',
            output='screen'
        )
    ])
