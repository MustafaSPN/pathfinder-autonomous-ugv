import asyncio
import math
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
from fastapi import FastAPI, WebSocket
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time


from robot_localization.srv import FromLL, ToLL
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix,Imu,NavSatStatus,PointCloud2
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import Twist,PoseStamped
from nav2_msgs.action import NavigateThroughPoses, FollowPath
from visualization_msgs.msg import Marker, MarkerArray

FRONTEND_DIR = (Path(__file__).parent.parent / "frontend").resolve()

app = FastAPI()
app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")


@app.get("/")
def root():
    return FileResponse(str(FRONTEND_DIR / "index.html"))


def quat_to_heading(x, y, z, w):
    """
    Quaternion (ENU, ROS standard) → Gerçek pusula heading (0–360°)
    """

    # 1️⃣ Quaternion → yaw (RAD)
    yaw = math.atan2(
        2.0 * (w*z + x*y),
        1.0 - 2.0 * (y*y + z*z)
    )

    # 2️⃣ Yaw → derece
    yaw_deg = math.degrees(yaw)

    # 3️⃣ Yaw → Heading dönüşümü
    heading_deg = (90.0 - yaw_deg) % 360.0

    return heading_deg

class WebBridgeNode(Node):
    def __init__(self):
        super().__init__("pathfinder_web_bridge")

        # Topics - change if needed
        self.gps_topic = self.declare_parameter("gps_topic", "/fix").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odometry/local").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel_stop").value
        self.heading_topic = self.declare_parameter("heading_topic", "/gps/imu").value

        # GOREV MODU:
        #   True  = NavigateThroughPoses (eski: NavFn planner + BT replan)
        #   False = FollowPath (YENI: planner YOK, WP'ler arasi birebir duz cizgi
        #           enterpole edilip dogrudan controller'a verilir -> replan/hipotenus
        #           kaynakli cizgiden sapma ortadan kalkar)
        self.use_planner_mission = self.declare_parameter("use_planner_mission", False).value
        self.fromll_client = self.create_client(FromLL, '/fromLL')
        self.toll_client = self.create_client(ToLL, '/toLL')
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.get_logger().info("FromLL and ToLL Service Clients initialized.")
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._heading_deg: float = 0.0
        self._vx: float = 0.0
        self._vyaw: float = 0.0

        # Nav2 Path storage
        self._current_path_ll: List[Dict[str, float]] = []

        # GPS status tracking (NavSatFix.status)
        self._gps_status_code: Optional[int] = None
        self._gps_status_service: Optional[int] = None
        self._gps_status_label: Optional[str] = None  # friendly label like 'RTK FIX', 'RTK Float', '3D FIX'
        self._gps_ts: Optional[float] = None  # time.time() of last received NavSatFix

        self._trace: List[Dict[str, float]] = []
        # 0.5m aralikli noktalarla 20000 nokta = ~10km iz. Clear butonuna
        # basilmadikca silinmez (eskiden 1000 nokta @20Hz = ~50sn'de siliniyordu).
        self._trace_max = 20000
        self._trace_min_dist_sq = 0.25  # 0.5m^2: bundan az hareket ettiyse nokta ekleme

        self._mission_state: str = "IDLE"

        # --- Lidar 2B engel katmani ---
        # Abonelik SADECE lidar sekmesi acikken kurulur (ws_lidar client sayaci);
        # yoksa 10Hz x 2.4MB PointCloud2 deserializasyonu Pi'de bosuna CPU yakar.
        self.lidar_topic = self.declare_parameter("lidar_topic", "/lidar_points").value
        # Sensore gore yukseklik filtresi: zemin donusunu at, engel bandini tut.
        # Lidar montaj yuksekligine gore ayarlanmali (z=0 sensor duzlemi).
        self._lidar_z_min = float(self.declare_parameter("lidar_z_min", -0.3).value)
        self._lidar_z_max = float(self.declare_parameter("lidar_z_max", 1.2).value)
        self._lidar_bins = 360           # 1 derece cozunurluk
        self._lidar_min_range = 0.5      # arac gövdesi/yakin donusler
        self._lidar_max_range = 30.0
        self._lidar_sub = None
        self._lidar_clients = 0
        self._lidar_last_proc = 0.0
        self._lidar_scan: Optional[List[float]] = None
        self._lidar_ts: Optional[float] = None

        self.create_subscription(NavSatFix, self.gps_topic, self._on_gps, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(Imu, self.heading_topic, self._on_heading, 10)  # just to keep the topic alive
        self.create_subscription(NavPath, '/plan', self._on_plan, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.cmd_vel_joy_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self._nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._follow_client = ActionClient(self, FollowPath, 'follow_path')
        # Duz cizgi modunda planner olmadigi icin /plan'i biz yayinliyoruz
        # (webui rota cizgisi + mission_logger bundan besleniyor)
        self.plan_pub = self.create_publisher(NavPath, '/plan', 10)
        self._current_goal_handle = None

        # Emergency stop state: when True publish zero cmd_vel continuously
        self._stopped: bool = False
        self._mission_state_before_stop: Optional[str] = None
        # timer to publish zero cmd_vel while stopped (10 Hz)
        self.create_timer(0.1, self._stopped_publisher_cb)

        self.get_logger().info(f"GPS topic: {self.gps_topic}")
        self.get_logger().info(f"Odom topic: {self.odom_topic}")
        self.get_logger().info(f"cmd_vel topic: {self.cmd_vel_topic}")

    def _on_gps(self, msg: NavSatFix):
        # Optional: ignore invalid fix (status < 0)
        self._lat = float(msg.latitude)
        self._lon = float(msg.longitude)

        # capture NavSatFix status (code & service) for frontend display
        try:
            self._gps_status_code = int(msg.status.status)
            self._gps_status_service = int(msg.status.service)
        except Exception:
            self._gps_status_code = None
            self._gps_status_service = None

        # Prefer RTD100 driver's semantics: explicit NAVSAT status codes indicate RTK
        label = None
        try:
            # Direct mapping from NavSatStatus when set by driver
            if self._gps_status_code == NavSatStatus.STATUS_GBAS_FIX:
                label = "RTK FIX"
            elif self._gps_status_code == NavSatStatus.STATUS_SBAS_FIX:
                label = "RTK Float"
            elif self._gps_status_code == NavSatStatus.STATUS_FIX:
                label = "3D FIX"
            elif self._gps_status_code == NavSatStatus.STATUS_NO_FIX:
                label = "No Fix"

            # Otherwise, fall back to covariance heuristic (driver sets covariance for BESTPOSA/GGA)
            if label is None:
                cov0 = None
                if hasattr(msg, 'position_covariance') and msg.position_covariance:
                    try:
                        cov0 = float(msg.position_covariance[0])
                    except Exception:
                        cov0 = None
                if cov0 is not None and cov0 > 0:
                    std_m = math.sqrt(cov0)
                    if std_m < 0.2:
                        label = "RTK FIX"
                    elif std_m < 3.0:
                        label = "RTK Float"

            # Final fallback
            if label is None:
                label = self._gps_status_text(self._gps_status_code)
        except Exception:
            label = self._gps_status_text(self._gps_status_code)

        # Debug log for diagnosis (will only show when logger level allows debug)
        try:
            cov0_debug = None
            if hasattr(msg, 'position_covariance') and msg.position_covariance:
                cov0_debug = float(msg.position_covariance[0])
            self.get_logger().debug(f"GPS label calc: code={self._gps_status_code}, svc={self._gps_status_service}, cov0={cov0_debug}, label={label}")
        except Exception:
            pass

        self._gps_status_label = label
        try:
            self._gps_ts = time.time()
        except Exception:
            self._gps_ts = None

        if self._lat and self._lon:
            # Mesafe filtresi: son noktadan >=0.5m hareket ettiyse ekle.
            # (GPS 20Hz; filtresiz buffer saniyeler icinde dolup eski izi siliyordu)
            add_point = True
            if self._trace:
                last = self._trace[-1]
                dlat_m = (self._lat - last["lat"]) * 111320.0
                dlon_m = (self._lon - last["lon"]) * 111320.0 * math.cos(math.radians(self._lat))
                add_point = (dlat_m * dlat_m + dlon_m * dlon_m) >= self._trace_min_dist_sq
            if add_point:
                self._trace.append({"lat": self._lat, "lon": self._lon})
                if len(self._trace) > self._trace_max:
                    self._trace = self._trace[-self._trace_max:]
    def lidar_client_add(self):
        self._lidar_clients += 1
        if self._lidar_sub is None:
            qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
            self._lidar_sub = self.create_subscription(
                PointCloud2, self.lidar_topic, self._on_lidar, qos)
            self.get_logger().info("Lidar aboneligi acildi (web client bagli).")

    def lidar_client_remove(self):
        self._lidar_clients = max(0, self._lidar_clients - 1)
        if self._lidar_clients == 0 and self._lidar_sub is not None:
            self.destroy_subscription(self._lidar_sub)
            self._lidar_sub = None
            self._lidar_scan = None
            self._lidar_ts = None
            self.get_logger().info("Lidar aboneligi kapatildi (client kalmadi).")

    def _on_lidar(self, msg: PointCloud2):
        now = time.monotonic()
        if now - self._lidar_last_proc < 0.2:  # en fazla 5 Hz isle
            return
        self._lidar_last_proc = now
        try:
            offs = {f.name: f.offset for f in msg.fields}
            dtype = np.dtype({'names': ['x', 'y', 'z'],
                              'formats': ['<f4', '<f4', '<f4'],
                              'offsets': [offs['x'], offs['y'], offs['z']],
                              'itemsize': msg.point_step})
            pts = np.frombuffer(msg.data, dtype=dtype)
            x, y, z = pts['x'], pts['y'], pts['z']

            m = (np.isfinite(x) & np.isfinite(y)
                 & (z >= self._lidar_z_min) & (z <= self._lidar_z_max))
            x, y = x[m], y[m]
            r = np.hypot(x, y)
            m2 = (r >= self._lidar_min_range) & (r <= self._lidar_max_range)
            x, y, r = x[m2], y[m2], r[m2]

            # Polar histogram: her 1 derecelik dilimde en yakin donus.
            # Bin i acisi: -pi + (i+0.5)*2pi/N (ROS: x ileri, y sol, CCW)
            nb = self._lidar_bins
            idx = ((np.arctan2(y, x) + np.pi) / (2.0 * np.pi) * nb).astype(np.int32) % nb
            scan = np.full(nb, np.inf, dtype=np.float32)
            np.minimum.at(scan, idx, r)
            scan[~np.isfinite(scan)] = 0.0  # 0 = o yonde engel yok
            self._lidar_scan = [round(float(v), 2) for v in scan]
            self._lidar_ts = time.time()
        except Exception as e:
            self.get_logger().warn(f"Lidar isleme hatasi: {e}")

    def _on_heading(self, msg: Imu):
        q = msg.orientation
        heading_deg = quat_to_heading(q.x, q.y, q.z, q.w)
        self._heading_deg = heading_deg

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # self._heading_deg = quat_to_heading_deg(q.x, q.y, q.z, q.w)
        self._vx = float(msg.twist.twist.linear.x)
        self._vyaw = float(msg.twist.twist.angular.z)

    def _on_plan(self, msg: NavPath):
        """Callback for Nav2 global plan updates."""
        # Safe way to bridge ROS callback thread -> asyncio loop
        try:
            loop = asyncio.get_event_loop()
            if loop.is_running():
                 asyncio.run_coroutine_threadsafe(self._process_plan(msg), loop)
            else:
                # Fallback if loop logic is tricky (rare case in FastAPI app context)
                pass
        except Exception:
            # If no loop is found in this context, we might skip
            pass

    async def _process_plan(self, msg: NavPath):
        if not self.toll_client.service_is_ready():
            return

        points = msg.poses
        if not points:
            self._current_path_ll = []
            return

        # Downsample: Take every 10th point to reduce service calls
        # Adjust step size based on path length if needed
        step = 15
        downsampled = points[::step]
        # Always include the last point
        if points[-1] not in downsampled:
            downsampled.append(points[-1])

        new_path_ll = []

        # We need to process these sequentially or in batches.
        # Making 50-100 service calls might be slow but it's the safest way
        # without duplicating robot_localization logic.

        for pose in downsampled:
            req = ToLL.Request()
            req.map_point.x = pose.pose.position.x
            req.map_point.y = pose.pose.position.y
            req.map_point.z = 0.0 # Assuming 2D plan on ground

            future = self.toll_client.call_async(req)
            # Polling wait
            while not future.done():
                await asyncio.sleep(0.005)

            try:
                res = future.result()
                new_path_ll.append({
                    "lat": res.ll_point.latitude,
                    "lon": res.ll_point.longitude
                })
            except Exception as e:
                self.get_logger().warn(f"ToLL service failed for point: {e}")

        self._current_path_ll = new_path_ll

    def _gps_status_text(self, code: Optional[int]) -> str:
        if code is None:
            return "UNKNOWN"
        mapping = {
            -1: "NO_FIX",
             0: "FIX",
             1: "SBAS_FIX",
             2: "GBAS_FIX",
        }
        return mapping.get(code, f"CODE:{code}")

    def clear_trace(self):
        """Clears the stored trace history."""
        self._trace = []
        self.get_logger().info("Trace history cleared.")

    def telemetry(self) -> Dict[str, Any]:
        lat = self._lat if self._lat is not None else 0.0
        lon = self._lon if self._lon is not None else 0.0
        heading_deg = self._heading_deg

        # Compute GPS label and age; report 'No GPS' if last message older than 1s
        gps_label = (self._gps_status_label if self._gps_status_label is not None else self._gps_status_text(self._gps_status_code))
        gps_age = None
        try:
            if self._gps_ts is not None:
                gps_age = time.time() - self._gps_ts
                if gps_age > 1.0:
                    gps_label = "No GPS"
        except Exception:
            pass

        return {
            "lat": lat,
            "lon": lon,
            "yaw_deg": heading_deg,
            "vx": self._vx,
            "vyaw": self._vyaw,
            "gps_status": {
                "code": self._gps_status_code,
                "service": self._gps_status_service,
                "str": gps_label,
                "age_s": round(gps_age, 3) if gps_age is not None else None,
            },
            "mission_state": self._mission_state,
            "trace": self._trace,  # tam iz; silme sadece Clear butonu ile
            "path": self._current_path_ll,
        }

    def emergency_stop(self):
        """Engage emergency stop: publish zero cmd_vel immediately and start continuous publishing."""
        msg = Twist()
        # publish immediate few messages for robustness
        for _ in range(5):
            self.cmd_pub.publish(msg)
        # set stopped flag so timer will continue publishing zeros
        if not self._stopped:
            self._mission_state_before_stop = self._mission_state
        self._stopped = True
        self._mission_state = "STOPPED"
        self.get_logger().warn("EMERGENCY STOP: engaged, publishing cmd_vel=0 continuously")

    def resume_stop(self):
        """Resume from emergency stop: stop continuous publishing and restore mission state."""
        if self._stopped:
            self._stopped = False
            self._mission_state = self._mission_state_before_stop or "IDLE"
            self._mission_state_before_stop = None
            self.get_logger().info("EMERGENCY STOP: resumed, stopped publishing cmd_vel")

    def _stopped_publisher_cb(self):
        """Timer callback that publishes zero cmd_vel while stopped."""
        if not self._stopped:
            return
        try:
            msg = Twist()
            self.cmd_pub.publish(msg)
        except Exception:
            self.get_logger().error("Failed to publish stop cmd_vel")
    def publish_markers(self, goal_msg):
        """
        Waypointleri RViz'de görmek için MarkerArray yayınlar.
        """
        marker_array = MarkerArray()

        # Eski markerları temizle (DELETEALL)
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Yeni noktaları ekle
        for i, pose in enumerate(goal_msg.poses):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "mission_waypoints"
            marker.id = i
            marker.type = Marker.SPHERE  # Noktaları küre olarak göster
            marker.action = Marker.ADD
            
            # Pozisyon
            marker.pose.position.x = pose.pose.position.x
            marker.pose.position.y = pose.pose.position.y
            marker.pose.position.z = 0.2  # Yerde kaybolmasın diye hafif yukarı
            
            # Boyut (0.5 metre çapında toplar)
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            # Renk (Cam Göbeği - Cyan)
            marker.color.a = 1.0 # Opaklık
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            
            marker_array.markers.append(marker)

            # Opsiyonel: Sıra numarasını gösteren yazı (TEXT)
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.ns = "mission_text"
            text_marker.id = 1000 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pose.pose.position.x
            text_marker.pose.position.y = pose.pose.position.y
            text_marker.pose.position.z = 0.8 # Topun üzerinde yazsın
            text_marker.scale.z = 0.4 # Yazı boyutu
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = str(i)
            
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info("RViz markerları yayınlandı.")
    async def get_map_coordinates(self, lat, lon):
        """
        /fromLL servisine güvenli asenkron çağrı yapar.
        ROS Future ile FastAPI async yapısını barıştırmak için polling kullanır.
        """
        # 1. Servis aktif mi kontrol et
        if not self.fromll_client.service_is_ready():
            self.get_logger().error("/fromLL servisi bulunamadı! 'ros2 run robot_localization navsat_transform_node' çalışıyor mu?")
            return None, None

        # 2. İsteği Hazırla
        req = FromLL.Request()
        req.ll_point = GeoPoint()
        req.ll_point.latitude = float(lat)
        req.ll_point.longitude = float(lon)
        req.ll_point.altitude = 0.0

        # 3. İsteği gönder (call_async)
        future = self.fromll_client.call_async(req)

        # 4. KRİTİK: ROS işlemini asyncio içinde bekletme (Polling)
        while not future.done():
            await asyncio.sleep(0.02) # 20ms bekle ve tekrar kontrol et

        # 5. Sonucu al
        try:
            result = future.result()
            return result.map_point.x, result.map_point.y
        except Exception as e:
            self.get_logger().error(f"Servis çağrısı hatası: {e}")
            return None, None

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _build_wp_poses(self, map_points):
        """WP listesinden, her biri sonraki noktaya bakan PoseStamped listesi."""
        poses = []
        for i, pt in enumerate(map_points):
            yaw = 0.0
            if i < len(map_points) - 1:
                nxt = map_points[i + 1]
                yaw = math.atan2(nxt['y'] - pt['y'], nxt['x'] - pt['x'])
            elif i > 0:
                prv = map_points[i - 1]
                yaw = math.atan2(pt['y'] - prv['y'], pt['x'] - prv['x'])
            poses.append(self._make_pose(pt['x'], pt['y'], yaw))
        return poses

    def _build_straight_path(self, map_points, step=0.3):
        """
        WP'ler arasi BIREBIR duz cizgi: poligonu 'step' araliklarla enterpole eder.
        Planner yok -> hipotenus/replan sapmasi yok; RPP cizgiden sapinca
        cizgiye GERI DONER (asil istenen davranis).
        """
        path = NavPath()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(map_points) - 1):
            a, b = map_points[i], map_points[i + 1]
            seg_dx, seg_dy = b['x'] - a['x'], b['y'] - a['y']
            seg_len = math.hypot(seg_dx, seg_dy)
            if seg_len < 1e-6:
                continue
            yaw = math.atan2(seg_dy, seg_dx)
            n = max(1, int(seg_len / step))
            for k in range(n):
                f = k / n
                path.poses.append(self._make_pose(a['x'] + f * seg_dx,
                                                  a['y'] + f * seg_dy, yaw))
        # son nokta
        last = map_points[-1]
        prev = map_points[-2] if len(map_points) > 1 else last
        end_yaw = math.atan2(last['y'] - prev['y'], last['x'] - prev['x'])
        path.poses.append(self._make_pose(last['x'], last['y'], end_yaw))
        return path

    async def send_mission_latlon(self, waypoints: List[Dict[str, float]]):
        """
        Waypoint listesini alır, dönüştürür ve göreve başlatır.
        use_planner_mission=True  -> NavigateThroughPoses (planner + BT)
        use_planner_mission=False -> FollowPath (duz cizgi, planner yok)
        """
        self.get_logger().info(f"{len(waypoints)} noktalı görev hazırlanıyor "
                               f"(mod: {'PLANNER' if self.use_planner_mission else 'DUZ CIZGI'})...")

        # 1. Tüm noktaları map koordinatına dönüştür
        map_points = []
        for i, wp in enumerate(waypoints):
            lat = wp.get('lat')
            lon = wp.get('lon')
            map_x, map_y = await self.get_map_coordinates(lat, lon)
            self.get_logger().info(f"Dönüşüm Kontrolü: Giriş(Lat:{lat}, Lon:{lon}) -> Çıkış(X:{map_x}, Y:{map_y})")
            if map_x is not None:
                map_points.append({'x': map_x, 'y': map_y})

        if not map_points:
            raise RuntimeError("Hiçbir waypoint geçerli bir harita koordinatına dönüştürülemedi!")

        wp_poses = self._build_wp_poses(map_points)

        # RViz/logger markerlari (her iki modda da WP'ler yayinlanir)
        from types import SimpleNamespace
        self.publish_markers(SimpleNamespace(poses=wp_poses))

        if self.use_planner_mission:
            # ============ ESKI MOD: NavigateThroughPoses ============
            if not self._nav_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("Nav2 Action Server (/navigate_through_poses) bulunamadı!")
                raise RuntimeError("Nav2 sistemi hazır değil. (Simülasyon veya Robot açık mı?)")

            goal_msg = NavigateThroughPoses.Goal()
            goal_msg.poses = wp_poses
            self.get_logger().info(f"Nav2'ye {len(wp_poses)} noktalı görev gönderiliyor (planner)...")
            send_goal_future = self._nav_client.send_goal_async(goal_msg)
        else:
            # ============ YENI MOD: FollowPath (duz cizgi) ============
            if not self._follow_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("FollowPath Action Server (/follow_path) bulunamadı!")
                raise RuntimeError("Controller server hazır değil. (Robot açık mı?)")

            # Robotun mevcut konumunu yolun basina ekle (GPS'ten cevir);
            # olmazsa yol ilk WP'den baslar (RPP en yakin noktadan devam eder).
            pts = list(map_points)
            if self._lat and self._lon:
                rx, ry = await self.get_map_coordinates(self._lat, self._lon)
                if rx is not None:
                    pts.insert(0, {'x': rx, 'y': ry})

            path = self._build_straight_path(pts)
            self.plan_pub.publish(path)   # webui + logger icin
            goal_msg = FollowPath.Goal()
            goal_msg.path = path
            goal_msg.controller_id = 'FollowPath'
            goal_msg.goal_checker_id = 'goal_checker'
            self.get_logger().info(
                f"Controller'a {len(path.poses)} noktalı DUZ CIZGI yolu gönderiliyor...")
            send_goal_future = self._follow_client.send_goal_async(goal_msg)

        while not send_goal_future.done():
            await asyncio.sleep(0.02)

        # Sonucu İşle
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Görev reddedildi!')
            self._mission_state = "MISSION_REJECTED"
            return

        self.get_logger().info('Görev kabul edildi, robot harekete başlıyor.')
        self._current_goal_handle = goal_handle
        self._mission_state = "MISSION_STARTED"

    def _mission_accepted_callback(self, future):
        """Nav2 görevi kabul etti mi etmedi mi burada anlarız ve Handle'ı saklarız."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Görev Nav2 tarafından reddedildi!')
            self._mission_state = "MISSION_REJECTED"
            return

        self.get_logger().info('Görev kabul edildi, robot harekete başlıyor.')
        # --- KRİTİK NOKTA: Handle'ı saklıyoruz ---
        self._current_goal_handle = goal_handle

    async def cancel_mission(self):
        self.get_logger().warn("İptal isteği alındı...")

        # 1. Aktif bir görev var mı kontrol et
        if self._current_goal_handle is None:
            self.get_logger().info("İptal edilecek aktif bir görev yok.")
            self._mission_state = "IDLE"
            return

        # 2. İptal isteğini Nav2'ye gönder
        self.get_logger().info("Nav2'ye iptal sinyali gönderiliyor...")
        future = self._current_goal_handle.cancel_goal_async()
        
        # İptal sonucunu beklemek için callback ekleyebiliriz (opsiyonel)
        future.add_done_callback(self._cancel_done_callback)
        
        self._mission_state = "CANCELLING..."

    def _cancel_done_callback(self, future):
        """İptal işlemi tamamlandığında çalışır"""
        self.get_logger().info("Görev başarıyla iptal edildi.")
        self._mission_state = "MISSION_CANCELLED"
        # Handle'ı sıfırla ki tekrar iptal etmeye çalışmayalım
        self._current_goal_handle = None

    def publish_cmd_vel_joy(self, linear_x: float, angular_z_deg: float):
        """Publish a Twist on /cmd_vel_joy. angular_z_deg is in deg/s, converted to rad/s."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z_deg) * math.pi / 180.0
        self.cmd_vel_joy_pub.publish(msg)

ros_node: Optional[WebBridgeNode] = None
ros_spin_task: Optional[asyncio.Task] = None


@app.on_event("startup")
async def on_startup():
    global ros_node, ros_spin_task
    rclpy.init(args=None)
    ros_node = WebBridgeNode()

    async def spin():
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.1)
            await asyncio.sleep(0.01)

    ros_spin_task = asyncio.get_event_loop().create_task(spin())


@app.on_event("shutdown")
async def on_shutdown():
    global ros_node, ros_spin_task
    if ros_spin_task:
        ros_spin_task.cancel()
    if ros_node:
        ros_node.destroy_node()
    rclpy.shutdown()


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await asyncio.sleep(0.1)  # 10 Hz
            await ws.send_json(ros_node.telemetry())
    except Exception:
        pass


@app.websocket("/ws_lidar")
async def ws_lidar_endpoint(ws: WebSocket):
    await ws.accept()
    ros_node.lidar_client_add()

    async def rx_config():
        # Istemciden canli filtre ayari: {"z_min": ..., "z_max": ...}
        # (zemin donusu filtresi; sensor yerde/montajda farkli deger ister)
        while True:
            data = await ws.receive_json()
            if "z_min" in data:
                ros_node._lidar_z_min = float(data["z_min"])
            if "z_max" in data:
                ros_node._lidar_z_max = float(data["z_max"])

    rx_task = asyncio.get_event_loop().create_task(rx_config())
    try:
        while True:
            await asyncio.sleep(0.2)  # 5 Hz
            age = (time.time() - ros_node._lidar_ts) if ros_node._lidar_ts else None
            await ws.send_json({
                "ranges": ros_node._lidar_scan,
                "bins": ros_node._lidar_bins,
                "min_range": ros_node._lidar_min_range,
                "max_range": ros_node._lidar_max_range,
                "z_min": ros_node._lidar_z_min,
                "z_max": ros_node._lidar_z_max,
                "age_s": round(age, 2) if age is not None else None,
            })
    except Exception:
        pass
    finally:
        rx_task.cancel()
        ros_node.lidar_client_remove()


@app.post("/api/stop")
async def api_stop():
    # Toggle emergency stop: if already stopped -> resume, else engage stop
    if ros_node._stopped:
        ros_node.resume_stop()
        return {"ok": True, "state": "resumed"}
    else:
        ros_node.emergency_stop()
        return {"ok": True, "state": "stopped"}


@app.post("/api/mission")
async def api_mission(payload: Dict[str, Any]):
    wps = payload.get("waypoints", [])
    if not isinstance(wps, list) or len(wps) == 0:
        return {"ok": False, "error": "No waypoints provided."}
    await ros_node.send_mission_latlon(wps)
    return {"ok": True}


@app.post("/api/cancel")
async def api_cancel():
    await ros_node.cancel_mission()
    return {"ok": True}


@app.post("/api/clear_trace")
async def api_clear_trace():
    if ros_node:
        ros_node.clear_trace()
    return {"ok": True}


@app.post("/api/cmd_vel_joy")
async def api_cmd_vel_joy(payload: Dict[str, Any]):
    linear = float(payload.get("linear", 0.0))
    angular = float(payload.get("angular", 0.0))
    # Auto-cancel any active Nav2 mission when manual input is sent
    if ros_node._current_goal_handle is not None:
        await ros_node.cancel_mission()
    ros_node.publish_cmd_vel_joy(linear, angular)
    return {"ok": True}