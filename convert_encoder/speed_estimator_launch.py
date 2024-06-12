from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='convert_encoder',  # 실제 패키지 이름으로 변경
            executable='speed_estimator',  # 실제 실행 파일 이름으로 변경
            name='speed_estimator',
            output='screen',
            parameters=[
                # 여기에 필요한 노드의 파라미터를 추가할 수 있습니다.
            ]
        ),
    ])

