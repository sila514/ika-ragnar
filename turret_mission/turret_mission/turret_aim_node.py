#!/usr/bin/env python3
"""Gorev 7: Gorsel hizalama / taret PID.

/detected_targets (vision_msgs/Detection2DArray) -> en yuksek skorlu hedefi al
piksel hatasi -> P kontrolcu -> /turret_cmd (geometry_msgs/Vector3, x=pan, y=tilt)
Hedef kayipsa: <=1s sabit tut (HOLDING), >1s arama moduna don (SEARCHING).
Hizalanmis (|hata|<tolerans) ve /estop==false ise /fire_cmd=true, 2s sonra false + reset.
"""
import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool


class State(Enum):
    SEARCHING = auto()
    TRACKING = auto()
    HOLDING = auto()
    FIRING = auto()


class TurretAimNode(Node):

    def __init__(self):
        super().__init__('turret_aim_node')

        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('kp_pan', 0.03)
        self.declare_parameter('kp_tilt', 0.03)
        self.declare_parameter('pixel_tolerance', 15.0)
        self.declare_parameter('lost_timeout', 1.0)
        self.declare_parameter('fire_duration', 2.0)
        self.declare_parameter('min_score', 0.3)
        self.declare_parameter('max_cmd', 5.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('search_amplitude', 2.0)
        self.declare_parameter('search_period', 4.0)

        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.kp_pan = self.get_parameter('kp_pan').value
        self.kp_tilt = self.get_parameter('kp_tilt').value
        self.pixel_tolerance = self.get_parameter('pixel_tolerance').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.fire_duration = self.get_parameter('fire_duration').value
        self.min_score = self.get_parameter('min_score').value
        self.max_cmd = self.get_parameter('max_cmd').value
        self.search_amplitude = self.get_parameter('search_amplitude').value
        self.search_period = self.get_parameter('search_period').value
        control_rate_hz = self.get_parameter('control_rate_hz').value

        # /estop hic yayinlanmadiysa guvenli varsayim: durdurulmus DEGIL.
        # (Sadece simulasyon icin gecerli bir varsayim; gercek donanimda
        #  estop bilinmiyorsa "durdurulmus" varsayilmalidir.)
        self.estop = False
        self.last_detection_time = None
        self.last_error = (0.0, 0.0)
        self.new_detection = False
        self.state = State.SEARCHING
        self.fire_start_time = None
        self.start_time = self.get_clock().now()

        self.sub_targets = self.create_subscription(
            Detection2DArray, '/detected_targets', self._targets_cb, 10)
        self.sub_estop = self.create_subscription(
            Bool, '/estop', self._estop_cb, 10)

        self.pub_turret_cmd = self.create_publisher(Vector3, '/turret_cmd', 10)
        self.pub_fire_cmd = self.create_publisher(Bool, '/fire_cmd', 10)

        period = 1.0 / control_rate_hz
        self.timer = self.create_timer(period, self._control_loop)

        self.get_logger().info('turret_aim_node basladi.')

    def _estop_cb(self, msg: Bool):
        self.estop = msg.data
        if self.estop and self.state == State.FIRING:
            self._stop_firing()

    def _targets_cb(self, msg: Detection2DArray):
        best = None
        best_score = -1.0
        for det in msg.detections:
            if not det.results:
                continue
            score = max(r.hypothesis.score for r in det.results)
            if score >= self.min_score and score > best_score:
                best_score = score
                best = det

        if best is None:
            return

        cx = best.bbox.center.position.x
        cy = best.bbox.center.position.y
        self.last_error = (cx - self.image_width / 2.0, cy - self.image_height / 2.0)
        self.last_detection_time = self.get_clock().now()
        self.new_detection = True

    def _seconds_since_last_detection(self):
        if self.last_detection_time is None:
            return math.inf
        dt = self.get_clock().now() - self.last_detection_time
        return dt.nanoseconds / 1e9

    def _control_loop(self):
        had_new_detection = self.new_detection
        self.new_detection = False
        lost_for = self._seconds_since_last_detection()

        if self.state == State.FIRING:
            self._service_firing()
            return

        if had_new_detection:
            err_x, err_y = self.last_error
            aligned = abs(err_x) < self.pixel_tolerance and abs(err_y) < self.pixel_tolerance
            if aligned and not self.estop:
                self._start_firing()
                return
            self.state = State.TRACKING
            pan = self._clamp(self.kp_pan * err_x)
            tilt = self._clamp(self.kp_tilt * err_y)
            self._publish_turret_cmd(pan, tilt)
            return

        if lost_for <= self.lost_timeout:
            self.state = State.HOLDING
            self._publish_turret_cmd(0.0, 0.0)
            return

        self.state = State.SEARCHING
        self._run_search()

    def _run_search(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        pan = self.search_amplitude * math.sin(2.0 * math.pi * t / self.search_period)
        self._publish_turret_cmd(pan, 0.0)

    def _start_firing(self):
        self.state = State.FIRING
        self.fire_start_time = self.get_clock().now()
        self._publish_turret_cmd(0.0, 0.0)
        msg = Bool()
        msg.data = True
        self.pub_fire_cmd.publish(msg)
        self.get_logger().info('Hedef hizalandi, estop kapali -> ATES.')

    def _service_firing(self):
        elapsed = (self.get_clock().now() - self.fire_start_time).nanoseconds / 1e9
        if self.estop or elapsed >= self.fire_duration:
            self._stop_firing()
        else:
            self._publish_turret_cmd(0.0, 0.0)

    def _stop_firing(self):
        msg = Bool()
        msg.data = False
        self.pub_fire_cmd.publish(msg)
        self.fire_start_time = None
        self.state = State.TRACKING
        self.get_logger().info('Ates sonlandi, reset.')

    def _clamp(self, value):
        return max(-self.max_cmd, min(self.max_cmd, value))

    def _publish_turret_cmd(self, pan, tilt):
        msg = Vector3()
        msg.x = float(pan)
        msg.y = float(tilt)
        msg.z = 0.0
        self.pub_turret_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurretAimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
