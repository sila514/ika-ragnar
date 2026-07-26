#!/usr/bin/env python3
"""
RAGNAR IKA - Mock CPU YOLO Node (Hailo donanimi OLMADAN test icin)

Bu node gercek yolo_node.py'nin (Hailo-8L) YERINE gecmez - o RPi/Hailo
donanimiyla ayni kalmali. Bu sadece Hailo'su olmayan gelistirme makinesinde
(bu laptop) /detected_targets pipeline'ini (turret_mission, mission_node)
ucdan uca test edebilmek icin CPU'da calisan bir mock.

Format sozlesmesi yolo_node.py ile BIREBIR AYNI:
- /image_raw (sensor_msgs/Image) dinlenir
- /detected_targets (vision_msgs/Detection2DArray) yayinlanir
- her karede yayinlanir (bos array dahil), header = goruntunun header'i
- Detection2D.bbox piksel koordinatlarinda (center.x/y, size_x/size_y)
- ObjectHypothesisWithPose.hypothesis.class_id / score dolduruluyor

Tespit yontemi (gercek donanim/model gerektirmez, salt CPU + OpenCV):
1) HOG+SVM ile insan tespiti (cv2'nin hazir varsayilan people detector'i)
2) Simulasyonda ortada insan modeli olmadigi icin (parkur.sdf'te sadece
   arac ve pist meshi var), HOG hicbir sey bulamazsa kontur tabanli genel
   "nesne" tespitine dusuluyor (Canny kenar + findContours + alan filtresi).
   Bu sayede Gazebo'da kameranin gordugu pist/nesneler de tespit edilip
   /detected_targets uzerinden gercekten veri akitilabiliyor.
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

CONF_THRESHOLD = 0.4
MIN_CONTOUR_AREA = 1500
MAX_CONTOUR_DETECTIONS = 3


class MockYoloNodeCpu(Node):
    def __init__(self):
        super().__init__("mock_yolo_node_cpu")

        self.declare_parameter("use_hog", True)
        self.declare_parameter("use_contour_fallback", True)
        self.declare_parameter("conf_threshold", CONF_THRESHOLD)
        self.declare_parameter("min_contour_area", MIN_CONTOUR_AREA)

        self.use_hog = self.get_parameter("use_hog").value
        self.use_contour_fallback = self.get_parameter("use_contour_fallback").value
        self.conf_threshold = self.get_parameter("conf_threshold").value
        self.min_contour_area = self.get_parameter("min_contour_area").value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/image_raw", self.cb, 1)
        self.pub = self.create_publisher(Detection2DArray, "/detected_targets", 10)

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self._frame_count = 0
        self._last_fps_log = time.time()

        self.get_logger().info(
            "mock_yolo_node_cpu basladi (CPU/OpenCV) - "
            "HOG insan tespiti"
            + (" + kontur fallback aktif." if self.use_contour_fallback else " (fallback kapali).")
        )

    def cb(self, msg: Image):
        t0 = time.time()
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        detections = []
        if self.use_hog:
            detections.extend(self._detect_people(frame))
        if self.use_contour_fallback and not detections:
            detections.extend(self._detect_contours(frame))

        det_array = Detection2DArray()
        det_array.header = msg.header
        det_array.detections = detections
        self.pub.publish(det_array)

        self._log_fps(t0)

    def _detect_people(self, frame):
        detections = []
        rects, weights = self.hog.detectMultiScale(
            frame, winStride=(8, 8), padding=(8, 8), scale=1.05
        )
        for (x, y, w, h), weight in zip(rects, weights):
            score = float(1.0 / (1.0 + np.exp(-float(weight))))
            if score < self.conf_threshold:
                continue
            detections.append(
                self._make_detection(x + w / 2.0, y + h / 2.0, w, h, "person", score)
            )
        return detections

    def _detect_contours(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 60, 150)
        edges = cv2.dilate(edges, None, iterations=2)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        frame_area = frame.shape[0] * frame.shape[1]
        candidates = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_contour_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            score = float(min(0.95, 0.5 + area / frame_area))
            candidates.append((area, x, y, w, h, score))

        candidates.sort(key=lambda c: c[0], reverse=True)
        detections = []
        for _area, x, y, w, h, score in candidates[:MAX_CONTOUR_DETECTIONS]:
            detections.append(
                self._make_detection(x + w / 2.0, y + h / 2.0, w, h, "object", score)
            )
        return detections

    def _make_detection(self, cx, cy, w, h, class_id, score):
        det = Detection2D()
        det.bbox.center.position.x = float(cx)
        det.bbox.center.position.y = float(cy)
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = class_id
        hyp.hypothesis.score = score
        det.results.append(hyp)
        return det

    def _log_fps(self, t0):
        self._frame_count += 1
        now = time.time()
        elapsed_since_log = now - self._last_fps_log
        if elapsed_since_log >= 5.0:
            fps = self._frame_count / elapsed_since_log
            infer_ms = (now - t0) * 1000
            self.get_logger().info(
                f"FPS: {fps:.1f}  |  son karenin islem suresi: {infer_ms:.1f}ms"
            )
            self._frame_count = 0
            self._last_fps_log = now


def main():
    rclpy.init()
    node = MockYoloNodeCpu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
