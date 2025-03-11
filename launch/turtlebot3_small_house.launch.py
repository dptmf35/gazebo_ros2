import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Gazebo ROS 패키지 경로
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    # aws-robomaker-small-house-world 패키지 경로
    pkg_share = FindPackageShare(package='aws_robomaker_small_house_world').find('aws_robomaker_small_house_world')

    # Small House World 파일 경로
    world_file_name = 'small_house.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    # Gazebo 모델 경로 설정 (Small House World의 모델 포함)
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # TurtleBot3 패키지 경로
    pkg_turtlebot3_gazebo = FindPackageShare(package='turtlebot3_gazebo').find('turtlebot3_gazebo')

    # Launch configuration 변수
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    # Launch 인자 선언
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient (GUI 표시 여부)')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')

    # Gazebo 서버 실행
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())

    # Gazebo 클라이언트 실행
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    # TurtleBot3 Burger 스폰
    start_turtlebot3_burger_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-topic', '/robot_description',
            '-x', '-3.5', '-y', '1.0', '-z', '0.01'  # 초기 위치 (거실 근처 예시)
        ],
        output='screen'
    )

    # Launch Description 생성
    ld = LaunchDescription()

    # Launch 옵션 추가
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # 액션 추가
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_turtlebot3_burger_cmd)

    return ld