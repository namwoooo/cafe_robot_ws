#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from dataclasses import dataclass, field
from typing import List, Optional

@dataclass
class TableZone:
    table_id: int
    image_bbox: List[float]
    world_bbox: List[float]
    label: str = ''

    def contains_image_point(self, cx, cy):
        x1, y1, x2, y2 = self.image_bbox
        return x1 <= cx <= x2 and y1 <= cy <= y2

TABLE_ZONES = [
    TableZone(table_id=1, image_bbox=[0,0,320,240],   world_bbox=[-3.0,1.0,-1.0,3.0],   label='Table 1'),
    TableZone(table_id=2, image_bbox=[320,0,640,240], world_bbox=[1.0,1.0,3.0,3.0],     label='Table 2'),
    TableZone(table_id=3, image_bbox=[0,240,320,480], world_bbox=[-3.0,-3.0,-1.0,-1.0], label='Table 3'),
    TableZone(table_id=4, image_bbox=[320,240,640,480],world_bbox=[1.0,-3.0,3.0,-1.0],  label='Table 4'),
]

class TableMappingNode(Node):
    def __init__(self):
        super().__init__('table_mapping_node')
        self.track_sub = self.create_subscription(String, '/tracking/tracked_objects', self.tracking_callback, 10)
        self.robot_pose_sub = self.create_subscription(String, '/robot/current_table', self.robot_table_callback, 10)
        self.table_det_pub = self.create_publisher(String, '/table/detections', 10)
        self.current_robot_table = None
        self.get_logger().info('TableMappingNode initialized')

    def robot_table_callback(self, msg):
        try:
            self.current_robot_table = json.loads(msg.data).get('table_id')
        except:
            pass

    def tracking_callback(self, msg):
        try:
            tracked_objects = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')
            return

        table_detections = {zone.table_id: [] for zone in TABLE_ZONES}
        for obj in tracked_objects:
            bbox = obj.get('bbox', [])
            if len(bbox) < 4:
                continue
            x1, y1, x2, y2 = bbox
            cx, cy = (x1+x2)/2, (y1+y2)/2
            for zone in TABLE_ZONES:
                if zone.contains_image_point(cx, cy):
                    table_detections[zone.table_id].append({
                        'track_id': obj.get('track_id'),
                        'class_name': obj.get('class_name'),
                        'confidence': obj.get('confidence'),
                        'bbox': bbox, 'center': [cx, cy],
                    })
                    break

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
