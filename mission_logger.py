#!/usr/bin/env python3
# ==========================================================================
# MISSION LOGGER - gorev sirasinda her seyi tek dosyaya kaydeder
#
# Kaydedilenler:
#   waypoints  : /waypoint_markers (webui gorev gonderince)
#   plan       : /plan (her replan; nokta listesi)
#   pose       : /odometry/global (map'te gercek konum+yaw, 10Hz)
#   local      : /odometry/local (controller'in gordugu, 5Hz)
#   wheel      : /odom_esp (vx, vyaw - VESC'ten olculen, 10Hz)
#   cmd        : /cmd_vel (nav2 cikisi, tam hiz)
#   cmd_out    : /cmd_vel_out (mux sonrasi ESP'ye giden, tam hiz)
#   gps        : /fix (RTK durumu + kovaryans, 1Hz)
#   heading    : /gps/imu yaw (5Hz)
#
# KULLANIM (docker icinde):
#   python3 /root/ros2_ws/mission_logger.py
#   -> once logger'i baslat, SONRA webui'den gorevi ver
#   -> gorev bitince Ctrl+C
#   Cikti: ros2_ws/mission_logs/mission_YYYYmmdd_HHMMSS.jsonl
#
# Format: her satir bir JSON: {"t": <epoch>, "k": "<tur>", ...}
# ==========================================================================

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from visualization_msgs.msg import MarkerArray

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mission_logs')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionLogger(Node):
    def __init__(self, fpath):
        super().__init__('mission_logger')
        self.f = open(fpath, 'w', buffering=1)  # satir bazli flush
        self.counts = {}
        self.last_write = {}   # topic bazli throttle

        self.create_subscription(MarkerArray, '/waypoint_markers', self.on_wps, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(Odometry, '/odometry/global', self.on_pose, 30)
        self.create_subscription(Odometry, '/odometry/local', self.on_local, 30)
        self.create_subscription(Odometry, '/odom_esp', self.on_wheel, 30)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 30)
        self.create_subscription(Twist, '/cmd_vel_out', self.on_cmd_out, 30)
        self.create_subscription(NavSatFix, '/fix', self.on_fix, 30)
        self.create_subscription(Imu, '/gps/imu', self.on_heading, 30)

        self.create_timer(5.0, self.status)
        self.get_logger().info('Logger hazir. Simdi webui\'den gorevi ver.')

    # --- yardimcilar ---
    def w(self, kind, data, min_dt=0.0):
        now = time.time()
        if min_dt > 0.0:
            last = self.last_write.get(kind, 0.0)
            if now - last < min_dt:
                return
            self.last_write[kind] = now
        rec = {'t': round(now, 3), 'k': kind}
        rec.update(data)
        self.f.write(json.dumps(rec) + '\n')
        self.counts[kind] = self.counts.get(kind, 0) + 1

    # --- callbackler ---
    def on_wps(self, msg):
        pts = [{'x': round(m.pose.position.x, 2), 'y': round(m.pose.position.y, 2)}
               for m in msg.markers if m.action == 0]  # ADD olanlar
        if pts:
            self.w('waypoints', {'pts': pts})
            self.get_logger().info(f'GOREV YAKALANDI: {len(pts)} waypoint')

    def on_plan(self, msg):
        # yol cok yogun olabilir; her 3. noktayi al, 2 ondalik yeter
        pts = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)]
               for p in msg.poses[::3]]
        self.w('plan', {'n': len(msg.poses), 'pts': pts}, min_dt=0.9)

    def on_pose(self, msg):
        p = msg.pose.pose
        self.w('pose', {
            'x': round(p.position.x, 3), 'y': round(p.position.y, 3),
            'yaw': round(yaw_of(p.orientation), 4),
            'vx': round(msg.twist.twist.linear.x, 3),
            'vyaw': round(msg.twist.twist.angular.z, 3),
        }, min_dt=0.1)

    def on_local(self, msg):
        p = msg.pose.pose
        self.w('local', {
            'x': round(p.position.x, 3), 'y': round(p.position.y, 3),
            'yaw': round(yaw_of(p.orientation), 4),
        }, min_dt=0.2)

    def on_wheel(self, msg):
        self.w('wheel', {
            'vx': round(msg.twist.twist.linear.x, 3),
            'vyaw': round(msg.twist.twist.angular.z, 3),
        }, min_dt=0.1)

    def on_cmd(self, msg):
        self.w('cmd', {'vx': round(msg.linear.x, 3),
                       'wz': round(msg.angular.z, 3)})

    def on_cmd_out(self, msg):
        self.w('cmd_out', {'vx': round(msg.linear.x, 3),
                           'wz': round(msg.angular.z, 3)})

    def on_fix(self, msg):
        self.w('gps', {
            'status': msg.status.status,
            'cov0': round(msg.position_covariance[0], 5),
            'lat': round(msg.latitude, 8), 'lon': round(msg.longitude, 8),
        }, min_dt=1.0)

    def on_heading(self, msg):
        self.w('heading', {'yaw': round(yaw_of(msg.orientation), 4)}, min_dt=0.2)

    def status(self):
        s = ' '.join(f'{k}:{v}' for k, v in sorted(self.counts.items()))
        self.get_logger().info(f'kayit: {s}')


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    fpath = os.path.join(LOG_DIR, time.strftime('mission_%Y%m%d_%H%M%S.jsonl'))
    rclpy.init()
    node = MissionLogger(fpath)
    print(f'\n>>> LOG: {fpath}\n>>> Gorevi simdi ver. Bitince Ctrl+C.\n')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        print(f'\n>>> Kayit tamamlandi: {fpath}')
        print('>>> Ozet:', node.counts)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
