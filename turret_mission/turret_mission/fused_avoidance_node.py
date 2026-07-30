#!/usr/bin/env python3
"""Lidar + kamera birlesik koni kacinma.

Iki ayri node'un (obstacle_avoidance_node, vision_avoidance_node) yerine
GECER, ikisini AYNI kararda birlestirir - artik ikisini birden calistirip
cmd_vel_nav'da catistirmaya gerek yok, TEK bu node calistirilmali.

Is bolumu:
- LIDAR (/scan) = GUVENLIK. On tarafta gercekten cok yakin bir sey varsa
  (koni olsun olmasin, duvar da olsa) once o durdurur/dondurur - vizyon
  yanilsa/koniyi kacirsa bile bu son savunma hatti.
- KAMERA (/detected_targets, 'koni' sinifi) = YONLENDIRME. Lidar acil
  durum gormuyorsa ama kamera koni(ler) goruyorsa, aralarindaki bosluga
  (iki koni sol/sag dengeliyse) ya da tek koniden uzaklasmaya yonelik
  direksiyon kamera tarafinca belirlenir.
- Kamera hicbir koni gormuyor ama lidar orta mesafede bir engel
  goruyorsa (duvar vs.): sadece lidar'a gore yavaslayip daha bos tarafa
  don (eski obstacle_avoidance_node'un fallback davranisi).
- Ikisi de temizse: sabit hizda ileri.

/scan ve /detected_targets hicbiri TAZE degilse (detection_timeout_s
icinde gelmemisse) GUVENLI TARAFTA KAL: dur.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist


class FusedAvoidanceNode(Node):

    def __init__(self):
        super().__init__('fused_avoidance_node')

        # --- lidar (guvenlik) parametreleri ---
        self.declare_parameter('front_half_angle_deg', 30.0)
        self.declare_parameter('stop_distance', 0.5)
        self.declare_parameter('caution_distance', 1.0)
        # --- kamera (yonlendirme) parametreleri ---
        self.declare_parameter('target_class_id', 'koni')
        self.declare_parameter('min_score', 0.3)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('panic_width_px', 220.0)
        self.declare_parameter('danger_width_px', 60.0)
        self.declare_parameter('gap_side_margin_px', 40.0)
        # --- ortak ---
        self.declare_parameter('cruise_speed', 0.15)
        self.declare_parameter('max_turn', 0.6)
        self.declare_parameter('sensor_timeout_s', 0.5)

        self.front_half_angle = math.radians(
            self.get_parameter('front_half_angle_deg').value)
        self.stop_distance = self.get_parameter('stop_distance').value
        self.caution_distance = self.get_parameter('caution_distance').value

        self.target_class_id = self.get_parameter('target_class_id').value
        self.min_score = self.get_parameter('min_score').value
        self.image_width = self.get_parameter('image_width').value
        self.panic_width = self.get_parameter('panic_width_px').value
        self.danger_width = self.get_parameter('danger_width_px').value
        self.gap_margin = self.get_parameter('gap_side_margin_px').value

        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.sensor_timeout_s = self.get_parameter('sensor_timeout_s').value

        self.pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.create_subscription(
            Detection2DArray, 'detected_targets', self._targets_cb, 10)
        self.create_timer(0.1, self._tick)  # 10Hz karar dongusu

        self._last_scan = None
        self._last_scan_time = None
        self._last_cones = []
        self._last_cones_time = None

        self.get_logger().info(
            f'fused_avoidance_node basladi (stop={self.stop_distance}m, '
            f'caution={self.caution_distance}m, cruise={self.cruise_speed}m/s).')

    # ------------------------------------------------------------------
    # Callbacks: sadece en son veriyi sakla, karar _tick()'te veriliyor.
    # ------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan):
        self._last_scan = msg
        self._last_scan_time = self.get_clock().now()

    def _targets_cb(self, msg: Detection2DArray):
        cones = []
        for det in msg.detections:
            for r in det.results:
                if r.hypothesis.class_id == self.target_class_id and r.hypothesis.score >= self.min_score:
                    cones.append(det)
                    break
        self._last_cones = cones
        self._last_cones_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # Lidar yardimcilari (obstacle_avoidance_node ile ayni mantik)
    # ------------------------------------------------------------------
    def _valid_ranges_in_window(self, msg: LaserScan, angle_lo, angle_hi):
        valid = []
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            if angle < angle_lo or angle > angle_hi:
                continue
            r = msg.ranges[i]
            if r <= msg.range_min or r > msg.range_max or math.isnan(r) or math.isinf(r):
                continue
            valid.append(r)
        return valid

    def _lidar_front_min(self):
        if self._last_scan is None:
            return None
        vals = self._valid_ranges_in_window(
            self._last_scan, -self.front_half_angle, self.front_half_angle)
        return min(vals) if vals else None

    def _lidar_clearer_side(self):
        """Sol mu sag mi daha bos, lidar'a gore. Pozitif -> sola don (CCW)."""
        msg = self._last_scan
        left = self._valid_ranges_in_window(
            msg, self.front_half_angle, self.front_half_angle + math.radians(60))
        right = self._valid_ranges_in_window(
            msg, -self.front_half_angle - math.radians(60), -self.front_half_angle)
        left_min = min(left) if left else 999.0
        right_min = min(right) if right else 999.0
        return self.max_turn if left_min >= right_min else -self.max_turn

    # ------------------------------------------------------------------
    # Kamera yardimcisi (vision_avoidance_node ile ayni mantik)
    # ------------------------------------------------------------------
    def _vision_cmd(self):
        cones = self._last_cones
        center_x = self.image_width / 2.0
        cmd = Twist()

        widest = max(cones, key=lambda d: d.bbox.size_x)
        widest_w = widest.bbox.size_x

        left_cones = [d for d in cones if d.bbox.center.position.x < center_x - self.gap_margin]
        right_cones = [d for d in cones if d.bbox.center.position.x > center_x + self.gap_margin]

        if left_cones and right_cones and widest_w < self.panic_width:
            nearest_left = max(left_cones, key=lambda d: d.bbox.size_x)
            nearest_right = max(right_cones, key=lambda d: d.bbox.size_x)
            gap_center = (nearest_left.bbox.center.position.x +
                          nearest_right.bbox.center.position.x) / 2.0
            error = gap_center - center_x
            cmd.angular.z = self._clamp(-0.01 * error, -self.max_turn, self.max_turn)
            cmd.linear.x = self.cruise_speed * 0.6
            return cmd

        error = widest.bbox.center.position.x - center_x

        if widest_w >= self.panic_width:
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_turn if error > 0 else -self.max_turn
        elif widest_w >= self.danger_width:
            closeness = min(1.0, widest_w / self.panic_width)
            cmd.linear.x = self.cruise_speed * (1.0 - closeness)
            avoid_dir = 1.0 if error > 0 else -1.0
            cmd.angular.z = self._clamp(avoid_dir * closeness * self.max_turn,
                                         -self.max_turn, self.max_turn)
        else:
            cmd.linear.x = self.cruise_speed
            cmd.angular.z = 0.0
        return cmd

    # ------------------------------------------------------------------
    # Ana karar dongusu
    # ------------------------------------------------------------------
    def _fresh(self, t):
        if t is None:
            return False
        return (self.get_clock().now() - t).nanoseconds / 1e9 <= self.sensor_timeout_s

    def _tick(self):
        scan_fresh = self._fresh(self._last_scan_time)
        cones_fresh = self._fresh(self._last_cones_time)

        if not scan_fresh:
            # Lidar'siz guvenlik garanti edilemez -> dur.
            self.pub.publish(Twist())
            return

        front_min = self._lidar_front_min()

        # 1) GUVENLIK: lidar cok yakinda bir sey goruyorsa, kamera ne
        #    derse desin dur ve daha bos tarafa don.
        if front_min is not None and front_min < self.stop_distance:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self._lidar_clearer_side()
            self.pub.publish(cmd)
            return

        # 2) Kamera koni goruyorsa: yonlendirmeyi kamera versin.
        if cones_fresh and self._last_cones:
            cmd = self._vision_cmd()
            # Lidar orta mesafede bir sey goruyorsa hizi ekstra sinirla
            # (kamera cok iyimser olabilir, lidar son sozu soylesin).
            if front_min is not None and front_min < self.caution_distance:
                cmd.linear.x = min(cmd.linear.x, self.cruise_speed * 0.4)
            self.pub.publish(cmd)
            return

        # 3) Kamera koni gormuyor ama lidar orta mesafede genel bir engel
        #    goruyorsa (duvar vs.): sadece lidar'a gore yavasla + don.
        if front_min is not None and front_min < self.caution_distance:
            cmd = Twist()
            closeness = 1.0 - (front_min - self.stop_distance) / max(
                1e-6, self.caution_distance - self.stop_distance)
            closeness = self._clamp(closeness, 0.0, 1.0)
            cmd.linear.x = self.cruise_speed * (1.0 - 0.7 * closeness)
            cmd.angular.z = self._lidar_clearer_side() * closeness
            self.pub.publish(cmd)
            return

        # 4) Her sey temiz: duz ileri.
        cmd = Twist()
        cmd.linear.x = self.cruise_speed
        cmd.angular.z = 0.0
        self.pub.publish(cmd)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = FusedAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
