#!/usr/bin/env python3
"""Minimal twist mux: /cmd_vel_joy (yuksek oncelik) + /cmd_vel_nav (dusuk oncelik) -> /cmd_vel.

ros-jazzy-twist-mux 4.5.0 bu ortamda hicbir mesaj yayinlamiyor (dogrulandi:
locks/QoS/use_sim_time hepsi elendi, debug loglarinda yarim kalmis "lol"
debug stringi var - paket kirik gibi duruyor). Bu basit alternatif ayni
onceliklendirme davranisini garantili sekilde saglar.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

JOY_TIMEOUT_S = 0.5
NAV_TIMEOUT_S = 0.5


class SimpleTwistMux(Node):

    def __init__(self):
        super().__init__('simple_twist_mux')
        self.last_joy = None
        self.last_joy_time = None
        self.last_nav = None
        self.last_nav_time = None

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_joy', self._joy_cb, 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self._nav_cb, 10)
        self.create_timer(0.05, self._tick)  # 20 Hz output
        self._last_published_zero = True
        self.get_logger().info('simple_twist_mux basladi (joy > nav oncelik).')

    def _joy_cb(self, msg: Twist):
        self.last_joy = msg
        self.last_joy_time = self.get_clock().now()

    def _nav_cb(self, msg: Twist):
        self.last_nav = msg
        self.last_nav_time = self.get_clock().now()

    def _fresh(self, t, timeout_s):
        if t is None:
            return False
        age = (self.get_clock().now() - t).nanoseconds / 1e9
        return age <= timeout_s

    def _tick(self):
        # Her tick'te MUTLAKA bir sey yayinla (joy > nav > sifir). Gazebo'nun
        # DiffDrive plugin'i son aldigi komutu sonsuza dek uygulamaya devam
        # eder - hicbir sey yayinlamamak, kaynak "stale" oldugunda aracin
        # son hizla surmeye devam etmesine yol acar (bu bug'i canli olarak
        # yasadik: 0.3 m/s komut kesildikten sonra arac durmadi).
        if self._fresh(self.last_joy_time, JOY_TIMEOUT_S):
            self.pub.publish(self.last_joy)
        elif self._fresh(self.last_nav_time, NAV_TIMEOUT_S):
            self.pub.publish(self.last_nav)
        else:
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTwistMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
