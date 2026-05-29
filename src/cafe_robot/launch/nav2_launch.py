import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_cafe = get_package_share_directory('cafe_robot')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_cafe, 'maps', 'cafe_map.yaml')
    nav2_params = os.path.join(pkg_cafe, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Nav2 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': nav2_params,
            }.items(),
        ),

        # 초기 위치 자동 설정
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl_init',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'initial_pose_x': 0.0,
                'initial_pose_y': 0.0,
                'initial_pose_a': 0.0,
            }],
        ),
    ])
