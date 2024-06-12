from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_geometry',
            executable='laser_geometry_node',
            name='laser_geometry_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            remappings=[
                ('/scan', '/your_laser_scan_topic'),
                ('/cloud', '/your_point_cloud_topic')
            ],
        ),
    ])
