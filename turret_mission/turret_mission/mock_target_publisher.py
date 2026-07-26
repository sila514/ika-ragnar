#!/usr/bin/env python3
"""Test amacli sahte /detected_targets yayinlayici (gercek Hailo/YOLO yerine).

/turret_cmd'yi dinler ve hedefin goruntudeki piksel konumunu, taretin
uyguladigi pan/tilt duzeltmesi kadar merkeze yaklastirarak simule eder
(kapali cevrim: hata azaldikca turret_cmd kucular, boylece PID yakinsamasi
gercekci sekilde test edilir). Hedef gorunurlugu periyodik olarak acilip
kapatilir (visible_duration / hidden_duration) ki HOLD/SEARCH mantigi da
test edilebilsin.
"""
import math

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Vector3


class MockTargetPublisher(Node):

    def __init__(self):
        super().__init__('mock_target_publisher')

        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('visible_duration', 6.0)
        self.declare_parameter('hidden_duration', 3.0)
        self.declare_parameter('sim_gain', 1.0)
        self.declare_parameter('score', 0.9)
        self.declare_parameter('initial_err_x', 180.0)
        self.declare_parameter('initial_err_y', -90.0)
        self.declare_parameter('mode', 'intermittent')  # 'intermittent' | 'static' | 'hidden'

        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.visible_duration = self.get_parameter('visible_duration').value
        self.hidden_duration = self.get_parameter('hidden_duration').value
        self.sim_gain = self.get_parameter('sim_gain').value
        self.score = self.get_parameter('score').value
        self.mode = self.get_parameter('mode').value

        self.err_x = self.get_parameter('initial_err_x').value
        self.err_y = self.get_parameter('initial_err_y').value

        self.sub_turret_cmd = self.create_subscription(
            Vector3, '/turret_cmd', self._turret_cmd_cb, 10)
        self.pub_targets = self.create_publisher(
            Detection2DArray, '/detected_targets', 10)

        self.start_time = self.get_clock().now()
        rate = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'mock_target_publisher basladi (mode={self.mode}, '
            f'gorunur={self.visible_duration}s, gizli={self.hidden_duration}s).')

    def _turret_cmd_cb(self, msg: Vector3):
        # Taret duzeltme uyguladikca hedefin goruntudeki hatasi azalir.
        self.err_x -= msg.x * self.sim_gain
        self.err_y -= msg.y * self.sim_gain
        half_w = self.image_width / 2.0
        half_h = self.image_height / 2.0
        self.err_x = max(-half_w, min(half_w, self.err_x))
        self.err_y = max(-half_h, min(half_h, self.err_y))

    def _is_visible_now(self):
        # Testler sirasinda "ros2 param set /mock_target_publisher mode ..."
        # ile canli degistirilebilsin diye her cagrida taze okunur.
        self.mode = self.get_parameter('mode').value
        if self.mode == 'hidden':
            return False
        if self.mode == 'static':
            return True
        cycle = self.visible_duration + self.hidden_duration
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        phase = math.fmod(t, cycle)
        was_hidden = getattr(self, '_was_hidden', True)
        visible = phase < self.visible_duration
        if visible and was_hidden:
            # Yeniden gorunur oldugunda hedef ekranda yeni/rastgele bir yerde
            # "belirir" -> PID'nin tekrar yakinsamasini test eder.
            self.err_x = self.get_parameter('initial_err_x').value
            self.err_y = self.get_parameter('initial_err_y').value
        self._was_hidden = not visible
        return visible

    def _tick(self):
        if not self._is_visible_now():
            return

        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        det = Detection2D()
        det.header = msg.header
        det.id = 'mock_target_1'

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = 'target'
        hyp.hypothesis.score = self.score
        det.results.append(hyp)

        det.bbox.center.position.x = self.image_width / 2.0 + self.err_x
        det.bbox.center.position.y = self.image_height / 2.0 + self.err_y
        det.bbox.size_x = 40.0
        det.bbox.size_y = 40.0

        msg.detections.append(det)
        self.pub_targets.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockTargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
