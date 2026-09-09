#!/bin/bash
# =============================================================
# FREEZE TESHIS - arac "Failed to make progress" verip durunca
# hangi sinyalin dondugunu/kestigini bulur.
#
# Container icinde, sistem + gorev ACIKKEN calistir:
#   docker exec -it ros2_humble bash /root/ros2_ws/diag_freeze.sh
#
# Her ~1 sn'de bir ozet satir basar. Araci hareket ettir, durdugu
# an ekrandaki FROZEN / NO-MSG isaretlerine bak:
#   * /odometry/global FROZEN ama /cmd_vel != 0  -> LOKALIZASYON dondu
#       (controller gitmesini soyluyor ama pozisyon guncellenmiyor)
#   * /cmd_vel == 0 (veya NO-MSG)                -> CONTROLLER durdu
#       (yol/costmap/goal sorunu; pozisyon donmuyorsa)
#   * /fix veya /gps/imu NO-MSG / FROZEN         -> SENSOR/GPS kesildi
#   * /cmd_vel != 0 ama /cmd_vel_out == 0        -> twist_mux/estop kesiyor
# =============================================================
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# Tek bir alani guvenli oku (topic kesikse timeout ile NO-MSG)
get() { timeout 2 ros2 topic echo "$1" --once --field "$2" 2>/dev/null | head -1; }

rtk_str() {
  case "$1" in
    -1) echo "NO_FIX" ;;
     0) echo "3D_FIX" ;;
     1) echo "RTK_FLOAT" ;;
     2) echo "RTK_FIX" ;;
     *) echo "?($1)" ;;
  esac
}

# fmt: deger bos ise NO-MSG, prev ile ayni ise *FROZEN* isareti
fmt() { # $1=val $2=prev
  if [ -z "$1" ]; then echo "NO-MSG"; elif [ "$1" == "$2" ]; then echo "$1 *FROZEN*"; else echo "$1"; fi
}

echo "FREEZE teshis basladi. Gorevi baslat. Durdurmak: Ctrl-C"
echo "-------------------------------------------------------------"

pgx=""; pgy=""; plat=""; plon=""; pimu=""; plx=""; ply=""
while true; do
  T=$(date +%H:%M:%S)

  gx=$(get /odometry/global pose.pose.position.x)
  gy=$(get /odometry/global pose.pose.position.y)
  lx=$(get /odometry/local  pose.pose.position.x)
  ly=$(get /odometry/local  pose.pose.position.y)
  lat=$(get /fix latitude)
  lon=$(get /fix longitude)
  st=$(get /fix status.status)
  imu=$(get /gps/imu orientation.z)
  cvx=$(get /cmd_vel linear.x)
  cwz=$(get /cmd_vel angular.z)
  ovx=$(get /cmd_vel_out linear.x)

  echo "[$T]"
  echo "  map EKF (global) : x=$(fmt "$gx" "$pgx")  y=$(fmt "$gy" "$pgy")"
  echo "  odom EKF (local) : x=$(fmt "$lx" "$plx")  y=$(fmt "$ly" "$ply")"
  echo "  GPS /fix         : lat=$(fmt "$lat" "$plat") lon=$(fmt "$lon" "$plon")  RTK=$(rtk_str "$st")"
  echo "  heading /gps/imu : qz=$(fmt "$imu" "$pimu")"
  echo "  nav2 /cmd_vel    : vx=${cvx:-NO-MSG}  wz=${cwz:-NO-MSG}"
  echo "  sürücü /cmd_vel_out: vx=${ovx:-NO-MSG}"

  pgx=$gx; pgy=$gy; plat=$lat; plon=$lon; pimu=$imu; plx=$lx; ply=$ly
  sleep 1
done
