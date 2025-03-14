import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    package_dir = get_package_share_directory('aws_robomaker_small_house_world')
    
    # 상대 경로 대신 패키지 경로를 사용
    world_path = os.path.join(package_dir, 'worlds', 'small_house.world')
    
    # 모델 경로 설정
    model_path = os.path.join(package_dir, 'models')
    
    # 현재 GAZEBO_MODEL_PATH 환경 변수 가져오기
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + model_path
    else:
        gazebo_model_path = model_path
    
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
        additional_env={'GAZEBO_MODEL_PATH': gazebo_model_path}  # 클라이언트에도 모델 경로 추가
    )

    # TurtleBot3 스폰 - 로봇 스테이션 앞에 위치하도록 변경
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
    
    # 텔레오퍼레이션 노드 추가
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        remappings=[
            ('/teleop_twist_keyboard/cmd_vel', '/cmd_vel'),
        ],
        # 새 터미널에서 실행하려면 아래 줄 주석 해제 (xterm 필요)
        # prefix='xterm -e',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run headless'
        ),
        gazebo_server,
        gazebo_client,
        spawn_turtlebot,
        teleop_node,  # 텔레오퍼레이션 노드 추가
    ])