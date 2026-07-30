#!/usr/bin/env python3
"""Basit reaktif engel kacinma (Nav2/harita GEREKTIRMEZ).

/scan (sensor_msgs/LaserScan) -> on taraftaki en yakin gecerli mesafeye
bakip /cmd_vel_nav (geometry_msgs/Twist) yayinlar. simple_twist_mux'taki
'joy > nav' onceligi sayesinde, joystick aktifken insan her zaman ustun
kalir (guvenlik); joystick bosta/yoksa bu node araci ileri surer ve
engelden (konidan) kacinir.

ONEMLI: LaserScan.ranges icindeki 0.0 degerler "gecersiz/donus yok"
anlamina gelir (menzil disi ya da bozuk paket) - "0 metrede engel var"
DEGIL. Bunlari filtrelemezsek node hicbir zaman hareket etmez (surekli
sahte "engel cok yakin" sanir). range_min'in altindaki degerler de ayni
sekilde disarida birakilir.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.declare_parameter('front_half_angle_deg', 30.0)
        self.declare_parameter('stop_distance', 0.5)
        self.declare_parameter('cruise_speed', 0.15)
        self.declare_parameter('turn_speed', 0.6)

        self.front_half_angle = math.radians(
            self.get_parameter('front_half_angle_deg').value)
        self.stop_distance = self.get_parameter('stop_distance').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value

        self.pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)

        self.get_logger().info(
            f'obstacle_avoidance_node basladi (stop_distance={self.stop_distance}m, '
            f'cruise_speed={self.cruise_speed}m/s).')

    def _valid_ranges_in_window(self, msg: LaserScan, angle_lo, angle_hi):
        """angle_lo/hi araligindaki GECERLI (0.0 ve range_min altindakiler
        haric) mesafeleri dondurur."""
        valid = []
        n = len(msg.ranges)
        for i in range(n):
            angle = msg.angle_min + i * msg.angle_increment
            if angle < angle_lo or angle > angle_hi:
                continue
            r = msg.ranges[i]
            if r <= msg.range_min or r > msg.range_max or math.isnan(r) or math.isinf(r):
                continue
            valid.append(r)
        return valid

    def _min_or_none(self, values):
        return min(values) if values else None

    def _scan_cb(self, msg: LaserScan):
        front = self._valid_ranges_in_window(
            msg, -self.front_half_angle, self.front_half_angle)
        front_min = self._min_or_none(front)

        cmd = Twist()

        if front_min is not None and front_min < self.stop_distance:
            # Engel yakin: dur, hangi tarafta daha fazla bosluk varsa o yone don.
            left = self._valid_ranges_in_window(
                msg, self.front_half_angle, self.front_half_angle + math.radians(60))
            right = self._valid_ranges_in_window(
                msg, -self.front_half_angle - math.radians(60), -self.front_half_angle)
            left_min = self._min_or_none(left)
            right_min = self._min_or_none(right)
            # Bilgi yoksa (None), o tarafi "acik" varsay (buyuk deger).
            left_clear = left_min if left_min is not None else 999.0
            right_clear = right_min if right_min is not None else 999.0

            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed if left_clear >= right_clear else -self.turn_speed
        else:
            cmd.linear.x = self.cruise_speed
            cmd.angular.z = 0.0

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
