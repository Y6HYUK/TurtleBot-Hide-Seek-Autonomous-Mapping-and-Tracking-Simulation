#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # turtlebot3_gazebo 패키지에서 turtlebot3_world.launch.py의 경로를 가져옵니다.
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_world_launch = os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')

    # turtlebot3_description 패키지 내에 있는 RViz 설정 파일 경로 (기본 모델 설정 파일 사용)
    tb3_description_dir = get_package_share_directory('turtlebot3_description')
    rviz_config = os.path.join(tb3_description_dir, 'rviz', 'model.rviz')

    return LaunchDescription([
        # Gazebo 실행: 기존 turtlebot3_world 런치파일 포함
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_world_launch)
        ),
        # RViz 실행: rviz2 명령어와 설정 파일 적용
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        )
    ])
