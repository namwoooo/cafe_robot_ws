#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

OBJECT_THRESHOLDS = {
    'cup': 6, 'bottle': 6, 'backpack': 3, 'handbag': 3,
    'suitcase': 3, 'laptop': 3, 'cell phone': 4, 'book': 4, 'default': 3,
}

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('camera_topic', '/camera/image_raw')

        model_path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value
        camera_topic = self.get_parameter('camera_topic').value

        self.bridge = CvBridge()

        if YOLO_AVAILABLE:
            self.model = YOLO(model_path)
            self.get_logger().info('YOLO model loaded')
        else:
            self.model = None
            self.get_logger().warn('ultralytics not installed. Dummy mode.')

        self.image_sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.detection_pub = self.create_publisher(String, '/yolo/detections', 10)
        self.annotated_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)
        self.get_logger().info('YoloNode initialized')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        detections = []
        if self.model is not None:
            results = self.model(cv_image, conf=self.confidence, verbose=False)
            for result in results:
                annotated = result.plot()
                ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                ann_msg.header = msg.header
                self.annotated_pub.publish(ann_msg)
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    detections.append({
                        'class_name': cls_name, 'confidence': conf,
                        'bbox': [x1, y1, x2, y2], 'timestamp': msg.header.stamp.sec,
                    })
        else:
            detections = self._dummy_detections(msg.header.stamp.sec)

        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.detection_pub.publish(det_msg)

    def _dummy_detections(self, timestamp):
        cycle = (timestamp // 10) % 2
        if cycle == 0:
            return [{'class_name': 'cup', 'confidence': 0.9,
                     'bbox': [100, 100, 200, 200], 'timestamp': timestamp}]
        return []

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
