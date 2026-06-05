#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from dataclasses import dataclass, field
from typing import Dict

OBJECT_THRESHOLDS = {
    'cup': 6, 'bottle': 6, 'wine glass': 6,
    'backpack': 3, 'handbag': 3, 'suitcase': 3,
    'laptop': 3, 'cell phone': 4, 'book': 4,
    'umbrella': 3, 'default': 3,
}
PERSON_CLASS = 'person'
ALERT_COOLDOWN = 3

def get_threshold(class_name):
    return OBJECT_THRESHOLDS.get(class_name, OBJECT_THRESHOLDS['default'])

@dataclass
class ObjectState:
    class_name: str
    count: int = 0
    last_seen_cycle: int = 0
    alerted: bool = False
    alert_cycle: int = 0

@dataclass
class TableState:
    table_id: int
    objects: Dict[str, ObjectState] = field(default_factory=dict)

    def abandoned_objects(self, current_cycle: int):
        result = []
        for o in self.objects.values():
            if o.count < get_threshold(o.class_name):
                continue
            if not o.alerted or (current_cycle - o.alert_cycle >= ALERT_COOLDOWN):
                result.append(o)
        return result


class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        self.declare_parameter('num_tables', 4)
        num_tables = self.get_parameter('num_tables').value

        self.table_states = {
            i: TableState(table_id=i) for i in range(1, num_tables + 1)
        }
        self.current_cycle = 0
        self.current_table = None
        self.counted_this_visit = False  # 테이블당 한 번만 카운트

        self.nav_event_sub = self.create_subscription(
            String, '/navigation/event', self.nav_event_callback, 10)
        self.tracking_sub = self.create_subscription(
            String, '/tracking/tracked_objects', self.tracking_callback, 10)
        self.alert_pub = self.create_publisher(String, '/alert/abandoned', 10)
        self.state_pub = self.create_publisher(String, '/state/summary', 10)

        self.get_logger().info(
            f'StateManagerNode initialized (alert_cooldown={ALERT_COOLDOWN} cycles)')

    def nav_event_callback(self, msg):
        try:
            event = json.loads(msg.data)
        except:
            return

        event_type = event.get('type')

        if event_type == 'cycle_start':
            self.current_cycle = event.get('cycle', self.current_cycle + 1)
            self.current_table = None
            self.counted_this_visit = False
            self.get_logger().info(f'=== Cycle {self.current_cycle} started ===')

        elif event_type == 'navigating_to_table':
            self.current_table = event.get('table_id')
            self.counted_this_visit = False
            self.get_logger().info(
                f'Cycle {self.current_cycle} | Navigating to table {self.current_table} -> Detection ON')

        elif event_type == 'arrived_table':
            self.counted_this_visit = False
            self.get_logger().info(
                f'Cycle {self.current_cycle} | Arrived at table {self.current_table}')

        elif event_type == 'arrived_counter':
            self.current_table = None
            self.counted_this_visit = False
            self._publish_all_alerts()

    def tracking_callback(self, msg):
        if self.current_table is None:
            return
        if self.counted_this_visit:
            return

        try:
            tracked_objects = json.loads(msg.data)
        except:
            return

        if not tracked_objects:
            return

        # 허용 클래스 필터링
        filtered = [o for o in tracked_objects if o['class_name'] != PERSON_CLASS]
        if not filtered:
            return

        self.counted_this_visit = True
        self.get_logger().info(
            f'Cycle {self.current_cycle} | Table {self.current_table} | '
            f'detected: {[o["class_name"] for o in filtered]}')
        self._update_table_state(self.current_table, filtered)
        self._publish_state_summary()

    def _update_table_state(self, table_id, detections):
        table = self.table_states[table_id]

        detected_classes = set()
        for det in detections:
            class_name = det['class_name']
            detected_classes.add(class_name)

            if class_name not in table.objects:
                table.objects[class_name] = ObjectState(
                    class_name=class_name,
                    count=0,
                    last_seen_cycle=self.current_cycle,
                )

            obj = table.objects[class_name]
            cycle_diff = self.current_cycle - obj.last_seen_cycle

            if cycle_diff <= 1:
                obj.count += 1
                obj.last_seen_cycle = self.current_cycle
                self.get_logger().info(
                    f'Table {table_id} | {class_name} '
                    f'count={obj.count}/{get_threshold(class_name)}')
            else:
                obj.count = 1
                obj.last_seen_cycle = self.current_cycle
                obj.alerted = False
                self.get_logger().info(
                    f'Table {table_id} | {class_name} reset '
                    f'(cycle gap={cycle_diff})')

        for class_name in list(table.objects.keys()):
            if class_name not in detected_classes:
                obj = table.objects[class_name]
                if self.current_cycle - obj.last_seen_cycle > 1:
                    del table.objects[class_name]

    def _publish_all_alerts(self):
        alerts = []
        for table_id, table in self.table_states.items():
            abandoned = table.abandoned_objects(self.current_cycle)
            if abandoned:
                alerts.append({
                    'table_id': table_id,
                    'objects': [
                        {'class_name': o.class_name, 'count': o.count}
                        for o in abandoned
                    ]
                })
                for obj in abandoned:
                    obj.alerted = True
                    obj.alert_cycle = self.current_cycle
                    self.get_logger().info(
                        f'Table {table_id} | {obj.class_name} alert sent '
                        f'(next alert after cycle {self.current_cycle + ALERT_COOLDOWN})')

        if alerts:
            msg = String()
            msg.data = json.dumps({
                'alerts': alerts,
                'cycle': self.current_cycle
            })
            self.alert_pub.publish(msg)
            self.get_logger().info(f'Alerts sent for {len(alerts)} tables')
        else:
            self.get_logger().info('No abandoned objects to report')

    def _publish_state_summary(self):
        summary = {
            tid: [
                {'class': o.class_name,
                 'count': o.count,
                 'threshold': get_threshold(o.class_name),
                 'alerted': o.alerted,
                 'alert_cycle': o.alert_cycle}
                for o in t.objects.values()
            ]
            for tid, t in self.table_states.items()
        }
        msg = String()
        msg.data = json.dumps({
            'cycle': self.current_cycle,
            'tables': summary
        })
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
