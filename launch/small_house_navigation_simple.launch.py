import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 소스 디렉토리 경로 (절대 경로 사용)
    source_dir = '/root/ros2_ws/src/aws-robomaker-small-house-world'
    
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
    
    # 텔레오퍼레이션 노드 추가
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        remappings=[
            ('/teleop_twist_keyboard/cmd_vel', '/cmd_vel'),
        ],
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
        gazebo_server,
        gazebo_client,
        spawn_turtlebot,
        teleop_node,
    ]) 