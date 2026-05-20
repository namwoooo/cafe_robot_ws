#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np

class SimpleTracker:
    def __init__(self, iou_threshold=0.3, max_lost=5):
        self.iou_threshold = iou_threshold
        self.max_lost = max_lost
        self.next_id = 1
        self.tracks = {}

    def update(self, detections):
        if not detections:
            to_delete = []
            for tid in self.tracks:
                self.tracks[tid]['lost'] += 1
                if self.tracks[tid]['lost'] > self.max_lost:
                    to_delete.append(tid)
            for tid in to_delete:
                del self.tracks[tid]
            return []

        det_bboxes = np.array([d['bbox'] for d in detections])
        matched_det = set()
        matched_track = set()
        results = []

        if self.tracks:
            track_ids = list(self.tracks.keys())
            track_bboxes = np.array([self.tracks[t]['bbox'] for t in track_ids])
            iou_matrix = self._iou_batch(track_bboxes, det_bboxes)

            for _ in range(min(len(track_ids), len(detections))):
                if iou_matrix.size == 0:
                    break
                max_idx = np.argmax(iou_matrix)
                ti, di = divmod(max_idx, iou_matrix.shape[1])
                if iou_matrix[ti, di] < self.iou_threshold:
                    break
                tid = track_ids[ti]
                matched_track.add(ti)
                matched_det.add(di)
                self.tracks[tid]['bbox'] = detections[di]['bbox']
                self.tracks[tid]['lost'] = 0
                results.append({'track_id': tid, 'class_name': detections[di]['class_name'],
                                'confidence': detections[di]['confidence'], 'bbox': detections[di]['bbox']})
                iou_matrix[ti, :] = -1
                iou_matrix[:, di] = -1

        to_delete = []
        for i, tid in enumerate(list(self.tracks.keys())):
            if i not in matched_track:
                self.tracks[tid]['lost'] += 1
                if self.tracks[tid]['lost'] > self.max_lost:
                    to_delete.append(tid)
        for tid in to_delete:
            del self.tracks[tid]

        for di, det in enumerate(detections):
            if di not in matched_det:
                tid = self.next_id
                self.next_id += 1
                self.tracks[tid] = {'bbox': det['bbox'], 'class_name': det['class_name'], 'lost': 0}
                results.append({'track_id': tid, 'class_name': det['class_name'],
                                'confidence': det['confidence'], 'bbox': det['bbox']})
        return results

    @staticmethod
    def _iou_batch(bboxes_a, bboxes_b):
        ax1,ay1,ax2,ay2 = bboxes_a[:,0],bboxes_a[:,1],bboxes_a[:,2],bboxes_a[:,3]
        bx1,by1,bx2,by2 = bboxes_b[:,0],bboxes_b[:,1],bboxes_b[:,2],bboxes_b[:,3]
        ix1 = np.maximum(ax1[:,None], bx1[None,:])
        iy1 = np.maximum(ay1[:,None], by1[None,:])
        ix2 = np.minimum(ax2[:,None], bx2[None,:])
        iy2 = np.minimum(ay2[:,None], by2[None,:])
        iw = np.maximum(0, ix2-ix1)
        ih = np.maximum(0, iy2-iy1)
        inter = iw * ih
        area_a = (ax2-ax1)*(ay2-ay1)
        area_b = (bx2-bx1)*(by2-by1)
        union = area_a[:,None] + area_b[None,:] - inter
        return inter / (union + 1e-6)

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        self.tracker = SimpleTracker()
        self.det_sub = self.create_subscription(String, '/yolo/detections', self.detection_callback, 10)
        self.track_pub = self.create_publisher(String, '/tracking/tracked_objects', 10)
        self.get_logger().info('TrackingNode initialized')

    def detection_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')
            return
        tracked = self.tracker.update(detections)
        track_msg = String()
        track_msg.data = json.dumps(tracked)
        self.track_pub.publish(track_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
