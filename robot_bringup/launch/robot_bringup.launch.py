# ROS 2 launch 관련 모듈 불러오기
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# launch 파일을 생성하는 함수
def generate_launch_description():


    # 1. main.cpp 실행 (다이나믹셀 모션 실행 제어)
    main_node = Node(
        package='forward_walk',      # 모션 실행 로직이 있는 패키지명
        executable='main_node',      # main.cpp에서 빌드된 실행파일 이름
        name='main_node',
        output='screen'
    )

    # 2. RealSense 카메라 실행 (별도의 launch 포함)
    realsense_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('realsense2_camera'),
                        'launch',
                        'rs_launch.py'
                    )
                )
            )
        ]
    )

    # 3. 12초 후 실행: 나머지 노드들
    delayed_nodes = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='my_cv',
                executable='line_publisher',
                name='line_publisher',
                output='screen'
            ),
            Node(
                package='my_cv',
                executable='line_subscriber',
                name='line_subscriber',
                output='screen'
            ),
            Node(
                package='decision',
                executable='motion_decision',
                name='motion_decision',
                output='screen'
            ),
        ]
    )



    # LaunchDescription 객체에 위에서 정의한 모든 노드 포함
    return LaunchDescription([
        main_node,
        realsense_launch,
        delayed_nodes
    ])