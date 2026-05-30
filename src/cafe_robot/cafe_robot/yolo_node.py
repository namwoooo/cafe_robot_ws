#!/usr/bin/env python3
"""
yolo_node.py
카메라 이미지를 수신하여 YOLOv8로 객체를 감지하고
감지 결과를 퍼블리시하는 노드

navigation_node의 이벤트와 연동:
- arrived_table 이벤트 수신 → 감지 시작
- arrived_counter 또는 cycle_start 이벤트 수신 → 감지 중지
"""

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

        # 감지 활성화 여부 (navigation 이벤트로 제어)
        self.detecting = False
        self.current_table = None

        if YOLO_AVAILABLE:
            self.model = YOLO(model_path)
            self.get_logger().info('YOLO model loaded')
        else:
            self.model = None
            self.get_logger().warn('ultralytics not installed. Dummy mode.')

        # Subscriber: 카메라 이미지
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)

        # Subscriber: 네비게이션 이벤트
        self.nav_event_sub = self.create_subscription(
            String, '/navigation/event', self.nav_event_callback, 10)

        # Publisher: 감지 결과
        self.detection_pub = self.create_publisher(String, '/yolo/detections', 10)
        self.annotated_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)

        self.get_logger().info('YoloNode initialized')

    def nav_event_callback(self, msg: String):
        """네비게이션 이벤트 수신 → 감지 활성화/비활성화"""
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        event_type = event.get('type')

        if event_type == 'arrived_table':
            # 테이블 도착 → 감지 시작
            self.current_table = event.get('table_id')
            self.detecting = True
            self.get_logger().info(
                f'Table {self.current_table} arrived → Detection ON')

        elif event_type in ('arrived_counter', 'cycle_start'):
            # 카운터 도착 또는 새 사이클 시작 → 감지 중지
            self.detecting = False
            self.current_table = None
            self.get_logger().info(
                f'Event [{event_type}] → Detection OFF')

    def image_callback(self, msg: Image):
        """카메라 이미지 콜백: 감지 활성화 상태일 때만 처리"""
        if not self.detecting:
            return

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
                        'class_name': cls_name,
                        'confidence': conf,
                        'bbox': [x1, y1, x2, y2],
                        'timestamp': msg.header.stamp.sec,
                    })
        else:
            # 더미 모드: 테이블 도착 시 1회만 감지
            detections = self._dummy_detections()

        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.detection_pub.publish(det_msg)

    def _dummy_detections(self):
        """더미 감지 결과 (테이블별로 다른 물체 반환)"""
        dummy_data = {
            1: [{'class_name': 'cup', 'confidence': 0.9,
                 'bbox': [100, 100, 200, 200], 'timestamp': 0}],
            2: [],  # 테이블 2 물체 없
            3: [{'class_name': 'backpack', 'confidence': 0.88,
                 'bbox': [120, 80, 280, 250], 'timestamp': 0}],
            4: [],  # 테이블 4 물체 없음
        }
        return dummy_data.get(self.current_table, [])


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
