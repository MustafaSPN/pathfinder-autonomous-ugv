#!/bin/bash
# GPS/EKF runaway teshis - sistem ACIKKEN container icinde calistir
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
echo "=================== /fix (ham GPS) ==================="
ros2 topic echo /fix --once --field status
ros2 topic echo /fix --once --field latitude
ros2 topic echo /fix --once --field longitude
echo "=================== /gps/imu (heading) ==============="
ros2 topic echo /gps/imu --once --field orientation
echo "=================== /odometry/gps (navsat cikti) ====="
ros2 topic echo /odometry/gps --once --field pose.pose.position
echo "=================== /odometry/global (map EKF) ======="
ros2 topic echo /odometry/global --once --field pose.pose.position
echo "=================== /odometry/local (odom EKF) ======="
ros2 topic echo /odometry/local --once --field pose.pose.position
echo "=================== TF map->base_link ================"
ros2 run tf2_ros tf2_echo map base_link --timeout 2 2>/dev/null | head -8
echo "=================== datum log ========================"
echo "(set_datum_auto loglarinda hangi lat/lon ile datum kuruldu, kontrol et)"
