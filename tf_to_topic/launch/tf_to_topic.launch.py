#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch system will look for
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf_to_topic',
            executable='tf_to_topic_node',
            name='tf_to_topic_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
    ])