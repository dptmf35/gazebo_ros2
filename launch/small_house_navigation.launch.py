import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 소스 디렉토리 경로 (절대 경로 사용)
    source_dir = '/root/ros2_ws/src/aws-robomaker-small-house-world'
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 맵 파일 경로 (절대 경로 사용)
    map_file = os.path.join(source_dir, 'maps/turtlebot3_waffle_pi', 'map.yaml')
    
    # 파라미터 파일 경로 (절대 경로 사용)
    nav2_params_file = os.path.join(source_dir, 'params', 'nav2_params.yaml')
    
    # 월드 파일 경로 (절대 경로 사용)
    world_path = os.path.join(source_dir, 'worlds', 'small_house.world')
    
    # 모델 경로 설정 (절대 경로 사용)
    model_path = os.path.join(source_dir, 'models')
    
    # 현재 GAZEBO_MODEL_PATH 환경 변수 가져오기
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + model_path
    else:
        gazebo_model_path = model_path
    
    # Launch 설정 파라미터
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Gazebo 서버 실행 (ROS 통합 포함)
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', world_path, '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': gazebo_model_path}
    )

    # Gazebo 클라이언트 실행 (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
        additional_env={'GAZEBO_MODEL_PATH': gazebo_model_path}
    )

    # TurtleBot3 스폰 - 로봇 스테이션 앞에 위치하도록 설정
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_waffle',
            '-x', '9.0', '-y', '0.8', '-z', '0.01', 
            '-R', '0', '-P', '0', '-Y', '1.57',
            '-file', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf'
        ],
        output='screen'
    )
    
    # 대신 개별 노드를 직접 실행
    
    # 맵 서버 노드
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'yaml_filename': map_file}]
    )
    
    # AMCL 노드
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file,
                    {'use_sim_time': use_sim_time}]
    )
    
    # 라이프사이클 매니저
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server', 'amcl']}]
    )
    
    # 컨트롤러 서버
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params_file,
                    {'use_sim_time': use_sim_time}]
    )
    
    # 플래너 서버
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file,
                    {'use_sim_time': use_sim_time}]
    )
    
    # 비헤이비어 서버 (Humble에서는 recoveries 대신 behaviors 사용)
    behaviors_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file,
                    {'use_sim_time': use_sim_time}]
    )
    
    # BT 네비게이터
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file,
                    {'use_sim_time': use_sim_time}]
    )
    
    # 웨이포인트 팔로워
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file,
                    {'use_sim_time': use_sim_time}]
    )
    
    # 네비게이션 라이프사이클 매니저
    nav_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['controller_server',
                                    'planner_server',
                                    'behavior_server',  # recoveries_server 대신 behavior_server 사용
                                    'bt_navigator',
                                    'waypoint_follower']}]
    )
    
    # 목표 지점 설정 노드 (TV 근처로 이동) - 지연 실행
    goal_node = Node(
        package='aws_robomaker_small_house_world',
        executable='scripts/set_goal.py',  # 실행 파일 경로 수정
        name='set_goal',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'goal_x': 0.8},  # TV 근처 x 좌표
            {'goal_y': -5.0},  # TV 근처 y 좌표
            {'goal_yaw': 0.0}  # 목표 방향
        ]
    )
    
    # 목표 지점 설정 노드는 일정 시간 후에 실행 (Nav2가 시작될 시간을 고려)
    delayed_goal = TimerAction(
        period=15.0,  # 15초 후 실행
        actions=[goal_node]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run headless'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'autostart', 
            default_value='true',
            description='Automatically start Nav2'
        ),
        gazebo_server,
        gazebo_client,
        spawn_turtlebot,
        map_server,
        amcl,
        lifecycle_manager,
        controller_server,
        planner_server,
        behaviors_server,  # recoveries_server 대신 behaviors_server 사용
        bt_navigator,
        waypoint_follower,
        nav_lifecycle_manager,
        delayed_goal  # 지연된 목표 지점 설정 노드
    ]) 