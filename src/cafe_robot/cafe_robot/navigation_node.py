#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json, math, time
from dataclasses import dataclass
from typing import List
from enum import Enum, auto

try:
    from nav2_msgs.action import NavigateToPose
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False

class RobotState(Enum):
    IDLE = auto()
    NAVIGATING = auto()

@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float = 0.0
    is_counter: bool = False
    table_id: int = -1

# waypoint 설정
WAYPOINTS: List[Waypoint] = [
    Waypoint(name='counter', x=0.0,  y=0.0,  yaw=0.0,   is_counter=True),
    Waypoint(name='table_1', x=-1.5, y=1.5,  yaw=-1.57, table_id=1),
    Waypoint(name='table_2', x=1.5,  y=1.5,  yaw=-1.57, table_id=2),
    Waypoint(name='table_3', x=-1.5, y=-1.5, yaw=1.57,  table_id=3),
    Waypoint(name='table_4', x=1.5,  y=-1.5, yaw=1.57,  table_id=4),
    Waypoint(name='counter', x=0.0,  y=0.0,  yaw=0.0,   is_counter=True),
]

def make_pose_stamped(wp):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = wp.x
    pose.pose.position.y = wp.y
    pose.pose.orientation.z = math.sin(wp.yaw / 2.0)
    pose.pose.orientation.w = math.cos(wp.yaw / 2.0)
    return pose

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.declare_parameter('patrol_interval', 10.0)
        self.declare_parameter('wait_at_table', 3.0)
        self.patrol_interval = self.get_parameter('patrol_interval').value
        self.wait_at_table = self.get_parameter('wait_at_table').value
        self.current_cycle = 0
        self.current_wp_index = 0
        self.robot_state = RobotState.IDLE

        self.nav_event_pub = self.create_publisher(String, '/navigation/event', 10)
        self.current_table_pub = self.create_publisher(String, '/robot/current_table', 10)

        if NAV2_AVAILABLE:
            self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        else:
            self._nav_client = None
            self.get_logger().warn('Nav2 not available. Timer simulation mode.')

        self._patrol_timer = self.create_timer(self.patrol_interval, self._start_patrol)
        self.get_logger().info('NavigationNode initialized')

    def _start_patrol(self):
        if self.robot_state == RobotState.NAVIGATING:
            return
        self.current_cycle += 1
        self.current_wp_index = 0
        self.robot_state = RobotState.NAVIGATING
        self._publish_event('cycle_start', cycle=self.current_cycle)
        self.get_logger().info(f'=== Patrol cycle {self.current_cycle} started ===')
        self._navigate_next()

    def _navigate_next(self):
        if self.current_wp_index >= len(WAYPOINTS):
            self.robot_state = RobotState.IDLE
            return
        wp = WAYPOINTS[self.current_wp_index]
        self.get_logger().info(f'Navigating to {wp.name} ({wp.x}, {wp.y})')
        
        # 다음 테이블로 이동 시작 시 이벤트 발행
        if not wp.is_counter:
            self._publish_event('navigating_to_table', cycle=self.current_cycle, table_id=wp.table_id)
        
        if NAV2_AVAILABLE and self._nav_client:
            self._send_nav2_goal(wp)
        else:
            self.create_timer(2.0, lambda: self._on_waypoint_arrived(wp))

    def _send_nav2_goal(self, wp):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(wp)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        self._nav_client.wait_for_server()
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._on_nav2_goal_response(f, wp))

    def _on_nav2_goal_response(self, future, wp):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_waypoint_arrived(wp))

    def _on_waypoint_arrived(self, wp):
        self.get_logger().info(f'Arrived at {wp.name}')
        if wp.is_counter:
            self._publish_event('arrived_counter', cycle=self.current_cycle)
        else:
            self._publish_event('arrived_table', cycle=self.current_cycle, table_id=wp.table_id)
            self._publish_current_table(wp.table_id)
            time.sleep(self.wait_at_table)
        self.current_wp_index += 1
        self._navigate_next()

    def _publish_event(self, event_type, **kwargs):
        msg = String()
        msg.data = json.dumps({'type': event_type, **kwargs})
        self.nav_event_pub.publish(msg)

    def _publish_current_table(self, table_id):
        msg = String()
        msg.data = json.dumps({'table_id': table_id})
        self.current_table_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
