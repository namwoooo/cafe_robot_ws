from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='yolov8n.pt'),
        DeclareLaunchArgument('confidence', default_value='0.5'),
        DeclareLaunchArgument('patrol_interval', default_value='10.0'),
        DeclareLaunchArgument('num_tables', default_value='4'),

        Node(package='cafe_robot', executable='yolo_node.py', name='yolo_node',
             output='screen',
             parameters=[{'model_path': LaunchConfiguration('model_path'),
                          'confidence': LaunchConfiguration('confidence'),
                          'camera_topic': '/camera/image_raw'}]),

        Node(package='cafe_robot', executable='tracking_node.py',
             name='tracking_node', output='screen'),

        Node(package='cafe_robot', executable='table_mapping_node.py',
             name='table_mapping_node', output='screen'),

        Node(package='cafe_robot', executable='state_manager_node.py',
             name='state_manager_node', output='screen',
             parameters=[{'num_tables': LaunchConfiguration('num_tables')}]),

        Node(package='cafe_robot', executable='alert_node.py',
             name='alert_node', output='screen'),

        TimerAction(period=5.0, actions=[
            Node(package='cafe_robot', executable='navigation_node.py',
                 name='navigation_node', output='screen',
                 parameters=[{'patrol_interval': LaunchConfiguration('patrol_interval'),
                              'wait_at_table': 3.0}])
        ]),
    ])
