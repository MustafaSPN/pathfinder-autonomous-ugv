#!/usr/bin/env python3
# ==========================================================================
# C_ERPM kalibrasyon olcumu (ROS tarafi - firmware flaslamaya gerek yok)
#
# Mantik:
#   /odom_esp  linear.x integrali  = odom_mesafe  (yanlis olcekli: ERPM/C_ERPM_current)
#   /odometry/global konum farki   = gercek_mesafe (RTK GPS ground-truth)
#   C_ERPM_yeni = C_ERPM_current * (odom_mesafe / gercek_mesafe)
#
# KULLANIM:
#   1. Rover + navsat + EKF ayakta olsun (rover.launch.py). GPS RTK FIX olsun.
#   2. python3 calibrate_cerpm.py
#   3. Joystick/teleop ile arabayi DUZ ~10-15 m sur (donus yok).
#   4. Durdur, Ctrl+C -> sonuc + onerilen C_ERPM ekrana basilir.
#   5. 2-3 kez tekrarla, ortalamayi firmware'deki #define C_ERPM'e yaz.
# ==========================================================================

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

C_ERPM_CURRENT = 5000.0   # firmware'de su an yazan deger


class CalibNode(Node):
    def __init__(self):
        super().__init__('cerpm_calib')

        # odom integrali
        self.odom_dist = 0.0
        self.last_odom_t = None

        # gps referans
        self.gps_start = None
        self.gps_last = None

        # ONEMLI: /odometry/gps = navsat_transform'un SAF GPS ciktisi (odom'dan
        # bagimsiz). /odometry/global ile kiyaslamak yaniltir: o fuzyon oldugu
        # icin GPS girmese bile odom'u tekrarlar ve sahte "dogru" verir.
        self._gps_msg_count = 0
        self.create_subscription(Odometry, '/odom_esp', self.on_odom, 20)
        self.create_subscription(Odometry, '/odometry/gps', self.on_gps, 20)
        self.create_timer(1.0, self.report)

        self.get_logger().info('Kalibrasyon basladi. Arabayi DUZ sur, bitince Ctrl+C.')

    def on_odom(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.last_odom_t is not None:
            dt = t - self.last_odom_t
            if 0.0 < dt < 1.0:  # makul aralik
                self.odom_dist += abs(msg.twist.twist.linear.x) * dt
        self.last_odom_t = t

    def on_gps(self, msg):
        self._gps_msg_count += 1
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.gps_start is None:
            self.gps_start = p
        self.gps_last = p

    def gps_dist(self):
        if self.gps_start is None or self.gps_last is None:
            return 0.0
        dx = self.gps_last[0] - self.gps_start[0]
        dy = self.gps_last[1] - self.gps_start[1]
        return math.hypot(dx, dy)

    def report(self):
        gd = self.gps_dist()
        self.get_logger().info(
            f'odom_mesafe={self.odom_dist:.2f} m | SAF-GPS={gd:.2f} m '
            f'| gps_msg={self._gps_msg_count}')

    def final(self):
        gd = self.gps_dist()
        print('\n========================================')
        print(f'  /odometry/gps mesaj sayisi    : {self._gps_msg_count}')
        if self._gps_msg_count == 0:
            print('  !!! /odometry/gps HIC GELMEDI !!!')
            print('  navsat_transform GPS uretmiyor -> GLOBAL EKF sadece odom')
            print('  dead-reckoning yapiyor. KOK SORUN BU. Datum/GPS FIX kontrol et.')
            print('========================================')
            return
        print(f'  odom integrali (yanlis olcek) : {self.odom_dist:.3f} m')
        print(f'  gercek mesafe (SAF GPS)       : {gd:.3f} m')
        if gd > 0.2 and self.odom_dist > 0.2:
            c_new = C_ERPM_CURRENT * (self.odom_dist / gd)
            print(f'  oran (odom/gercek)            : {self.odom_dist/gd:.3f}')
            print(f'  >>> ONERILEN C_ERPM           = {c_new:.1f}')
            print(f'      (su anki: {C_ERPM_CURRENT:.0f})')
        else:
            print('  YETERSIZ VERI: en az ~1 m duz surus gerekli. Tekrar dene.')
        print('========================================')


def main():
    rclpy.init()
    node = CalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.final()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
