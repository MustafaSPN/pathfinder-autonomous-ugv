#!/bin/bash
# Global EKF taze config + kacis kontrolu. Container icinde, sistem ACIKKEN calistir:
#   docker exec ros2_humble bash /root/ros2_ws/verify_ekf.sh
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

echo "### 1) ekf_filter_node_map node bekleniyor..."
for i in $(seq 1 15); do
  ros2 node list 2>/dev/null | grep -q ekf_filter_node_map && break
  sleep 1
done
ros2 node list 2>/dev/null | grep -q ekf_filter_node_map || { echo "HATA: ekf_filter_node_map yok. rover.launch.py acik mi?"; exit 1; }

echo "### 2) Calisan imu0_config (CANLI param):"
CFG=$(ros2 param get /ekf_filter_node_map imu0_config 2>/dev/null)
echo "    $CFG"
# 13. eleman = ax. Liste icindeki 'true' sayisi 1 ise (sadece vyaw) -> dogru.
TRUES=$(echo "$CFG" | grep -o "True" | wc -l)
if [ "$TRUES" -le 1 ]; then
  echo "    -> AX KAPALI (dogru, taze config)"
else
  echo "    -> !!! AX HALA ACIK / ESKI CONFIG. Sistem tam restart edilmemis."
fi

echo "### 3) /odometry/global kacis testi (4 sn):"
P1=$(timeout 3 ros2 topic echo /odometry/global --once --field pose.pose.position 2>/dev/null | tr -d "\n")
sleep 4
P2=$(timeout 3 ros2 topic echo /odometry/global --once --field pose.pose.position 2>/dev/null | tr -d "\n")
echo "    t0: $P1"
echo "    t4: $P2"

echo "### 4) /odometry/gps (navsat - referans ~0 olmali):"
timeout 3 ros2 topic echo /odometry/gps --once --field pose.pose.position 2>/dev/null | tr -d "\n"; echo
echo "### 4b) /odometry/gps frame_id (MAP olmali, odom DEGIL):"
FR=$(timeout 3 ros2 topic echo /odometry/gps --once 2>/dev/null | grep -m1 frame_id | tr -d " ")
echo "    $FR"
echo "$FR" | grep -q "map" && echo "    -> DOGRU (map frame)" || echo "    -> !!! HALA 'odom' -> runaway devam eder. Launch restart edilmemis."

echo "### VERDICT:"
echo "  - imu0_config 'True' sayisi: $TRUES  (1 olmali)"
echo "  - global pozisyon t0 vs t4 ayni kaliyorsa SORUN COZULDU."
echo "  - hala buyuyorsa: AX kapaliysa neden datum-jump; logu paylas."
