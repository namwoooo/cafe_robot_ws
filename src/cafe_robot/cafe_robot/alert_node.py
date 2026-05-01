#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime

CLASS_KO = {
    'cup':'컵', 'bottle':'병', 'wine glass':'와인잔',
    'backpack':'배낭', 'handbag':'핸드백', 'suitcase':'캐리어',
    'laptop':'노트북', 'cell phone':'휴대폰', 'book':'책', 'umbrella':'우산',
}

class AlertNode(Node):
    def __init__(self):
        super().__init__('alert_node')
        self.alert_sub = self.create_subscription(String, '/alert/abandoned', self.alert_callback, 10)
        self.alert_log_pub = self.create_publisher(String, '/alert/log', 10)
        self.get_logger().info('AlertNode initialized')

    def alert_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except:
            return
        alerts = data.get('alerts', [])
        cycle = data.get('cycle', '?')
        timestamp = datetime.now().strftime('%H:%M:%S')
        if not alerts:
            return

        self.get_logger().warn('=' * 50)
        self.get_logger().warn(f'  방치 물건 알림  [Cycle {cycle}] {timestamp}')
        self.get_logger().warn('=' * 50)

        log_entries = []
        for alert in alerts:
            table_id = alert['table_id']
            objects = alert.get('objects', [])
            obj_names = ', '.join(
                f"{CLASS_KO.get(o['class_name'], o['class_name'])}(감지횟수:{o['count']}회)"
                for o in objects)
            message = f'{table_id}번 테이블에 방치된 물건이 있습니다: {obj_names}'
            self.get_logger().warn(f'  ⚠️  {message}')
            log_entries.append({'table_id':table_id,'message':message,
                                'objects':objects,'timestamp':timestamp,'cycle':cycle})

        self.get_logger().warn('=' * 50)
        log_msg = String()
        log_msg.data = json.dumps(log_entries)
        self.alert_log_pub.publish(log_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AlertNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
