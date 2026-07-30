#!/usr/bin/env python3
"""Kamera/YOLO tabanli koni kacinma (lidar YOK, /detected_targets kullanir).

Monokuler kamerada gercek mesafe olcumu yok - bunun yerine tespit
kutusunun (bbox) GENISLIGINI yakinlik, MERKEZ KONUMUNU yon gostergesi
olarak kullaniyoruz: buyuk+ortadaki kutu = yakin ve tam onde tehlike,
kucuk/kenardaki kutu = uzak/az tehlike.

Davranis:
- Hic koni yoksa: sabit hizda ileri.
- Iki koni SOL ve SAG'da dengeli duruyorsa (aralarindan gecilebilecek bir
  bosluk varsa): ikisinin ORTA NOKTASINA dogru direksiyon (aralarindan
  gecmeye calisir).
- Tek/en tehlikeli koni (en genis kutu) belirleyici ise: o koninin
  TERSI yone direksiyon yapip yavaslar; kutu genisligi 'panic_width'i
  gecerse tamamen durup o yonden uzaklasana kadar doner.

/cmd_vel_nav'a yayinlar (simple_twist_mux: joystick aktifken insan
ustun kalir, bosta bu node devreye girer).

ONEMLI: turret_mission/obstacle_avoidance_node.py (lidar tabanli) ile AYNI
ANDA calistirmayin - ikisi de cmd_vel_nav'a yazar, komutlar carpisir/
titresebilir. Sadece birini secin (bu node kamera odakli istek icin).
"""
import math

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist


class VisionAvoidanceNode(Node):

    def __init__(self):
        super().__init__('vision_avoidance_node')

        self.declare_parameter('target_class_id', 'koni')
        self.declare_parameter('min_score', 0.3)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('cruise_speed', 0.15)
        self.declare_parameter('max_turn', 0.6)
        self.declare_parameter('panic_width_px', 220.0)  # bu genislikten buyukse -> ACIL
        self.declare_parameter('danger_width_px', 60.0)  # bu genislikten buyukse -> kacinma baslar
        self.declare_parameter('gap_side_margin_px', 40.0)  # iki koninin "dengeli" sayilmasi icin merkeze yakinlik toleransi
        self.declare_parameter('detection_timeout_s', 0.5)

        self.target_class_id = self.get_parameter('target_class_id').value
        self.min_score = self.get_parameter('min_score').value
        self.image_width = self.get_parameter('image_width').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.panic_width = self.get_parameter('panic_width_px').value
        self.danger_width = self.get_parameter('danger_width_px').value
        self.gap_margin = self.get_parameter('gap_side_margin_px').value
        self.detection_timeout_s = self.get_parameter('detection_timeout_s').value

        self.pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.create_subscription(
            Detection2DArray, 'detected_targets', self._targets_cb, 10)
        self.create_timer(0.1, self._tick)  # 10Hz - detection gelmese bile "cruise"a don

        self._last_cmd = Twist()
        self._last_detection_time = None

        self.get_logger().info(
            f'vision_avoidance_node basladi (panic={self.panic_width}px, '
            f'danger={self.danger_width}px, cruise={self.cruise_speed}m/s).')

    def _cones(self, msg: Detection2DArray):
        cones = []
        for det in msg.detections:
            for r in det.results:
                if r.hypothesis.class_id == self.target_class_id and r.hypothesis.score >= self.min_score:
                    cones.append(det)
                    break
        return cones

    def _targets_cb(self, msg: Detection2DArray):
        self._last_detection_time = self.get_clock().now()
        cones = self._cones(msg)
        cmd = Twist()
        center_x = self.image_width / 2.0

        if not cones:
            cmd.linear.x = self.cruise_speed
            cmd.angular.z = 0.0
            self._last_cmd = cmd
            return

        # En genis (en yakin sayilan) koniyi bul.
        widest = max(cones, key=lambda d: d.bbox.size_x)
        widest_w = widest.bbox.size_x

        # Aralarindan gecebilecegimiz bir "kapi" var mi: biri belirgin
        # sekilde soldaki, biri belirgin sekilde sagdaki, ikisi de panik
        # seviyesinde degil.
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
            self._last_cmd = cmd
            return

        # Kapi yok / tek koni: en tehlikeli koniden uzaklas.
        error = widest.bbox.center.position.x - center_x  # +: koni sagda, -: koni solda

        if widest_w >= self.panic_width:
            # Cok yakin: dur, koninin TERSI yone don (koni sagdaysa sola don -> +z).
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_turn if error > 0 else -self.max_turn
        elif widest_w >= self.danger_width:
            # Yaklasiyor: yavasla, hafifce ters yone kir.
            closeness = min(1.0, widest_w / self.panic_width)
            cmd.linear.x = self.cruise_speed * (1.0 - closeness)
            # error>0 -> koni sagda -> sola don (angular.z pozitif = CCW = sol).
            avoid_dir = 1.0 if error > 0 else -1.0
            cmd.angular.z = self._clamp(avoid_dir * closeness * self.max_turn,
                                         -self.max_turn, self.max_turn)
        else:
            # Uzak, henuz tehlike degil: duz devam.
            cmd.linear.x = self.cruise_speed
            cmd.angular.z = 0.0

        self._last_cmd = cmd

    def _tick(self):
        # /detected_targets bir sure gelmezse (kamera/yolo_node dustu vs.)
        # guvenli tarafta kal: dur.
        if self._last_detection_time is not None:
            age = (self.get_clock().now() - self._last_detection_time).nanoseconds / 1e9
            if age > self.detection_timeout_s:
                self.pub.publish(Twist())
                return
        self.pub.publish(self._last_cmd)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = VisionAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
