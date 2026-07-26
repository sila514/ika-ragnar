#!/usr/bin/env python3
"""Gorev 8: Gorev akisi.

nav2_simple_commander ile parkur waypoint'lerini sirayla Nav2'ye verir.
/detected_targets uzerinde (skoru esik ustu) bir hedef gorulunce mevcut
Nav2 gorevini iptal edip duraklatir (taret hizalama + ates dugumu -
turret_aim_node - kendi basina /detected_targets ve /estop'u dinleyip
hizalanip atesliyor). Hedef kaybolunca (ates tamamlandi VEYA hedef
suresince gorunmuyor) ayni waypoint'e navigasyona devam eder.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quaternion(yaw_rad):
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def make_pose(navigator, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    qx, qy, qz, qw = yaw_to_quaternion(math.radians(yaw_deg))
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class MissionNavigator(BasicNavigator):

    def __init__(self):
        super().__init__(node_name='mission_navigator')

        self.declare_parameter('min_score', 0.3)
        self.declare_parameter('engage_lost_timeout', 2.0)
        # Robotun Gazebo'daki baslangic (spawn) pozu - AMCL'e ilk poz olarak
        # verilir. parkur_arac.sdf'teki spawn pose'u ile eslesmelidir.
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', -12.0)
        self.declare_parameter('start_yaw_deg', 90.0)
        # Duz liste: [x1, y1, yaw1_deg, x2, y2, yaw2_deg, ...] - "map" frame'i.
        # maps/parkur_map.pgm'den (geodesic mesafe + duvar-mesafesi agirlikli
        # merkez hatti) cikarilan GERCEK parkur koridoru, ~4m araliklarla.
        # NOT: robotun parkur_arac.sdf'teki spawn poz'u (0,-12) bu koridorun
        # ~9m disinda kaliyor (bkz. config/mission_params.yaml notu) - harita
        # ile dunya dosyasi tam hizali degil, kullanmadan once dogrulayin.
        default_wp = [-7.14, -35.87, 109.3,
                      -8.44, -32.18, 99.3,
                      -9.06, -28.38, 116.2,
                      -11.07, -24.31, 90.3,
                      -11.09, -20.29, 86.5,
                      -10.84, -16.3, 67.2,
                      -9.15, -12.28, 63.3,
                      -7.15, -8.29, 66.0,
                      -5.41, -4.38, 47.2,
                      -1.59, -0.26, 66.9,
                      0.1, 3.71, 69.0,
                      1.64, 7.73, 74.2,
                      2.76, 11.7, 65.8,
                      4.55, 15.67, 95.4,
                      4.2, 19.31, 95.4]
        self.declare_parameter('waypoints', default_wp)

        self.min_score = self.get_parameter('min_score').value
        self.engage_lost_timeout = self.get_parameter('engage_lost_timeout').value
        # /detected_targets kesilirse (hedef kayip) bu sure icinde hala
        # "gorunur" sayilir - dedektorun kendi yayin periyoduna gore ayarlanir.
        self.declare_parameter('detection_freshness', 0.5)
        self.detection_freshness = self.get_parameter('detection_freshness').value

        self.last_detection_time = None
        self._prev_fire_cmd = False
        self.fire_completed_event = False

        self.create_subscription(
            Detection2DArray, '/detected_targets', self._targets_cb, 10)
        self.create_subscription(Bool, '/fire_cmd', self._fire_cb, 10)

    def get_waypoints(self):
        flat = self.get_parameter('waypoints').value
        if len(flat) % 3 != 0:
            self.get_logger().error(
                'waypoints parametresi 3 katı uzunlukta olmali [x,y,yaw,...]')
            return []
        return [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

    def _targets_cb(self, msg: Detection2DArray):
        best_score = -1.0
        for det in msg.detections:
            if not det.results:
                continue
            score = max(r.hypothesis.score for r in det.results)
            best_score = max(best_score, score)

        if best_score >= self.min_score:
            self.last_detection_time = self.get_clock().now()

    def _fire_cb(self, msg: Bool):
        if self._prev_fire_cmd and not msg.data:
            self.fire_completed_event = True
        self._prev_fire_cmd = msg.data

    def seconds_since_last_detection(self):
        if self.last_detection_time is None:
            return math.inf
        dt = self.get_clock().now() - self.last_detection_time
        return dt.nanoseconds / 1e9

    def is_target_visible(self):
        return self.seconds_since_last_detection() <= self.detection_freshness

    def engage_and_wait(self):
        """Hedef hizalanip atilana ya da kaybolana kadar bekler (bloklar)."""
        self.fire_completed_event = False
        self.get_logger().info('Hedef tespit edildi -> navigasyon duraklatildi, hizalama bekleniyor.')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.fire_completed_event:
                self.get_logger().info('Angajman tamamlandi (ates edildi).')
                return
            if self.seconds_since_last_detection() > self.engage_lost_timeout:
                self.get_logger().info('Hedef kayboldu, angajman iptal, navigasyona devam.')
                return


def run_mission(navigator: MissionNavigator):
    waypoints = navigator.get_waypoints()
    if not waypoints:
        navigator.get_logger().error('Waypoint listesi bos, gorev iptal.')
        return

    start_x = navigator.get_parameter('start_x').value
    start_y = navigator.get_parameter('start_y').value
    start_yaw_deg = navigator.get_parameter('start_yaw_deg').value
    navigator.setInitialPose(make_pose(navigator, start_x, start_y, start_yaw_deg))

    navigator.get_logger().info('Nav2 aktif olana kadar bekleniyor...')
    navigator.waitUntilNav2Active()

    for idx, (x, y, yaw_deg) in enumerate(waypoints):
        pose = make_pose(navigator, x, y, yaw_deg)
        navigator.get_logger().info(
            f'Waypoint {idx + 1}/{len(waypoints)} -> x={x}, y={y}, yaw={yaw_deg} derece')
        navigator.goToPose(pose)

        while not navigator.isTaskComplete():
            if navigator.is_target_visible():
                navigator.cancelTask()
                navigator.engage_and_wait()
                navigator.get_logger().info('Ayni waypoint icin navigasyona devam ediliyor.')
                navigator.goToPose(pose)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info(f'Waypoint {idx + 1} basariyla ulasildi.')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn(f'Waypoint {idx + 1} iptal edildi, sonrakine geciliyor.')
        else:
            navigator.get_logger().warn(
                f'Waypoint {idx + 1} basarisiz (result={result}), sonrakine geciliyor.')

    navigator.get_logger().info('Gorev tamamlandi: tum waypointler islendi.')


def main(args=None):
    rclpy.init(args=args)
    navigator = MissionNavigator()
    try:
        run_mission(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
