import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_cafe = get_package_share_directory('cafe_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_cafe, 'worlds', 'cafe_world.world')
    sdf_file = os.path.join(pkg_cafe, 'models', 'turtlebot3_burger_camera.sdf')
    urdf_file = os.path.join(pkg_cafe, 'urdf', 'turtlebot3_burger_camera.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # gzserver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
        ),

        # gzclient (GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # 로봇 spawn
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_burger',
                '-file', sdf_file,
                '-x', '0.0', '-y', '0.0', '-z', '0.01',
            ],
            output='screen',
        ),
    ])
