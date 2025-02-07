from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
import os

def generate_launch_description():
    # 더미 맵 파일 경로 지정
    my_pkg_share = get_package_share_directory('my_turtlebot_project')
    dummy_map = os.path.join(my_pkg_share, 'maps', 'dummy_map.yaml')
    print("Dummy map file path:", dummy_map)  # 디버깅용

    # "map" 인자를 더미 맵 파일로 기본값 설정 (이 인자는 실제로 사용하지 않더라도 필요)
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=dummy_map,
        description='Full path to map yaml file to load'
    )

    # Gazebo 실행: turtlebot3_gazebo의 world launch 파일 사용
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_world_launch = os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')

    # SLAM 실행: slam_toolbox의 online_async_launch.py 사용
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    # 네비게이션 실행: nav2_bringup의 bringup_launch.py 사용
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    # 기본 파라미터 파일 경로 (실제 파일 경로는 nav2_bringup 설치 상태에 따라 달라질 수 있음)
    params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')

    # RViz 실행: turtlebot3_description의 RViz 설정 파일 사용
    tb3_description_dir = get_package_share_directory('turtlebot3_description')
    rviz_config = os.path.join(tb3_description_dir, 'rviz', 'model.rviz')

    # 추가 터틀봇(예: turtlebot3_2)을 위한 robot_state_publisher 노드 그룹을 별도 네임스페이스로 실행
    # 여기서는 turtlebot3_burger의 URDF (xacro) 파일을 사용합니다.
    # (필요에 따라 추가 로봇에 맞는 컨트롤러나 기타 노드도 함께 실행해야 합니다.)
    additional_robot = GroupAction(
        actions=[
            PushRosNamespace('turtlebot3_2'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': Command([
                        'xacro ',
                        os.path.join(tb3_description_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
                    ])
                }]
            )
            # 추가로 컨트롤러나 기타 노드를 실행하려면 이곳에 Node 액션을 추가하세요.
        ]
    )

    return LaunchDescription([
        map_arg,
        # 기본 터틀봇, Gazebo, SLAM, Nav2, RViz 실행
        IncludeLaunchDescription(PythonLaunchDescriptionSource(tb3_world_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={'map': dummy_map, 'params_file': params_file}.items()
        ),
        ExecuteProcess(cmd=['rviz2', '-d', rviz_config], output='screen'),
        # 초기 위치 메시지를 자동으로 publish하는 ExecuteProcess를 TimerAction으로 지연 실행 (예: 5초 후)
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '--once', '/initialpose',
                        'geometry_msgs/PoseWithCovarianceStamped',
                        '{"header": {"frame_id": "map"}, '
                        '"pose": {"pose": {"position": {"x": 1.0, "y": 1.0, "z": 0.0}, '
                        '"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}, '
                        '"covariance": [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}'
                    ],
                    output='screen',
                    shell=True
                )
            ]
        ),
        # 추가 터틀봇 관련 그룹 액션 실행
        additional_robot
    ])
