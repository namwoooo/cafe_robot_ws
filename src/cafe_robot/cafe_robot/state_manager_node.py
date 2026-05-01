#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from dataclasses import dataclass, field
from typing import Dict

OBJECT_THRESHOLDS = {
    'cup':6, 'bottle':6, 'wine glass':6,
    'backpack':3, 'handbag':3, 'suitcase':3,
    'laptop':3, 'cell phone':4, 'book':4, 'umbrella':3, 'default':3,
}
PERSON_CLASS = 'person'

def get_threshold(class_name):
    return OBJECT_THRESHOLDS.get(class_name, OBJECT_THRESHOLDS['default'])

@dataclass
class ObjectState:
    track_id: int
    class_name: str
    count: int = 0
    last_seen_cycle: int = 0
    alerted: bool = False

@dataclass
class TableState:
    table_id: int
    objects: Dict[int, ObjectState] = field(default_factory=dict)

    def has_person(self):
        return any(o.class_name == PERSON_CLASS for o in self.objects.values())

    def abandoned_objects(self):
        return [o for o in self.objects.values()
                if not o.alerted and o.count >= get_threshold(o.class_name)]

class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        self.declare_parameter('num_tables', 4)
        num_tables = self.get_parameter('num_tables').value
        self.table_states = {i: TableState(table_id=i) for i in range(1, num_tables+1)}
        self.current_cycle = 0

        self.table_det_sub = self.create_subscription(String, '/table/detections', self.table_detection_callback, 10)
        self.nav_event_sub = self.create_subscription(String, '/navigation/event', self.nav_event_callback, 10)
        self.alert_pub = self.create_publisher(String, '/alert/abandoned', 10)
        self.state_pub = self.create_publisher(String, '/state/summary', 10)
        self.get_logger().info('StateManagerNode initialized')

    def nav_event_callback(self, msg):
        try:
            event = json.loads(msg.data)
        except:
            return
        if event.get('type') == 'cycle_start':
            self.current_cycle = event.get('cycle', self.current_cycle+1)
            self.get_logger().info(f'=== Cycle {self.current_cycle} started ===')
        elif event.get('type') == 'arrived_counter':
            self._publish_all_alerts()

    def table_detection_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except:
            return
        current_robot_table = data.get('current_robot_table')
        if current_robot_table is None:
            return
        table_id = int(current_robot_table)
        if table_id not in self.table_states:
            return
        detections = data.get('table_detections', {}).get(str(table_id), [])
        self._update_table_state(table_id, detections)
        self._publish_state_summary()

    def _update_table_state(self, table_id, detections):
        table = self.table_states[table_id]
        current_track_ids = {d['track_id'] for d in detections}
        person_detected = any(d['class_name'] == PERSON_CLASS for d in detections)

        for det in detections:
            track_id = det['track_id']
            class_name = det['class_name']
            if class_name == PERSON_CLASS:
                continue
            if track_id not in table.objects:
                table.objects[track_id] = ObjectState(
                    track_id=track_id, class_name=class_name,
                    count=0, last_seen_cycle=self.current_cycle)
            obj = table.objects[track_id]
            if not person_detected:
                cycle_diff = self.current_cycle - obj.last_seen_cycle
                if cycle_diff <= 1:
                    obj.count += 1
                    obj.last_seen_cycle = self.current_cycle
                    self.get_logger().info(
                        f'Table {table_id} | [{track_id}]{class_name} count={obj.count}/{get_threshold(class_name)}')
                else:
                    obj.count = 1
                    obj.last_seen_cycle = self.current_cycle
                    self.get_logger().info(f'Table {table_id} | [{track_id}]{class_name} reset')

        for track_id in list(table.objects.keys()):
            if track_id not in current_track_ids:
                obj = table.objects[track_id]
                if self.current_cycle - obj.last_seen_cycle > 1:
                    del table.objects[track_id]

    def _publish_all_alerts(self):
        alerts = []
        for table_id, table in self.table_states.items():
            abandoned = table.abandoned_objects()
            if abandoned:
                alerts.append({
                    'table_id': table_id,
                    'objects': [{'track_id':o.track_id,'class_name':o.class_name,'count':o.count}
                                for o in abandoned]
                })
                for obj in abandoned:
                    obj.alerted = True
                    obj.count = 0
        if alerts:
            alert_msg = String()
            alert_msg.data = json.dumps({'alerts': alerts, 'cycle': self.current_cycle})
            self.alert_pub.publish(alert_msg)

    def _publish_state_summary(self):
        summary = {tid: [{'track_id':o.track_id,'class':o.class_name,
                          'count':o.count,'threshold':get_threshold(o.class_name)}
                         for o in t.objects.values()]
                   for tid, t in self.table_states.items()}
        msg = String()
        msg.data = json.dumps({'cycle': self.current_cycle, 'tables': summary})
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
