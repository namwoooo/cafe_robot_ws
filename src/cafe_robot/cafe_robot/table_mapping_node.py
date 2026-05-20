#!/usr/bin/env python3
"""
table_mapping_node.py
navigation_node에서 /robot/current_table 토픽을 받아
현재 로봇이 위치한 테이블의 감지 결과만 state_manager로 전달
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class TableMappingNode(Node):
    def __init__(self):
        super().__init__('table_mapping_node')

        self.current_robot_table = None

        self.track_sub = self.create_subscription(
            String, '/tracking/tracked_objects', self.tracking_callback, 10)

        self.robot_table_sub = self.create_subscription(
            String, '/robot/current_table', self.robot_table_callback, 10)

        self.table_det_pub = self.create_publisher(
            String, '/table/detections', 10)

        self.get_logger().info('TableMappingNode initialized (waypoint-based)')

    def robot_table_callback(self, msg):
        try:
            data = json.loads(msg.data)
            new_table = data.get('table_id')
            if new_table != self.current_robot_table:
                self.get_logger().info(f'Robot moved to table {new_table}')
                self.current_robot_table = new_table
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')

    def tracking_callback(self, msg):
        try:
            tracked_objects = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')
            return

        if self.current_robot_table is None:
            return

        table_detections = {str(self.current_robot_table): tracked_objects}

        if tracked_objects:
            self.get_logger().info(
                f'Table {self.current_robot_table}: '
                f'{len(tracked_objects)} objects detected '
                + ', '.join(
                    f"[{o.get('track_id')}]{o.get('class_name')}"
                    for o in tracked_objects))

        out_msg = String()
        out_msg.data = json.dumps({
            'table_detections': table_detections,
            'current_robot_table': self.current_robot_table,
        })
        self.table_det_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TableMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
