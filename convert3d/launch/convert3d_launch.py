#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# this is the function launch system will look for
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='convert3d',
            executable='convert3d_node',
            name='convert3d_node',
            output='screen',
            parameters=[{
                'use_sim_time': False  # 시뮬레이션 시간을 사용할 경우 True로 설정
            }],
        )
    ])
