import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from robot_localization.srv import SetDatum
import math


class AutoDatumSetter(Node):
    """
    Datum'u (map 0,0 referansi) SADECE guvenilir GPS ile kurar.

    Eski surum: status >= 0 olan ILK fix'i datum yapiyordu. RTK kilitlenmeden
    gelen tek bir cop fix (ornegin Null Island 0,0 ya da metrelerce sapan Single
    fix) datum'a oturunca, gercek fix gelince aradaki fark yuzlerce/binlerce
    metre olup arac map frame'de "uzaklasip out-of-bounds" oluyordu.

    Yeni surum 3 sart arar:
      1) status >= min_fix_status  (varsayilan 2 = RTK FIXED / GBAS)
      2) lat/lon gecerli  (NaN/inf yok, Null Island yok, aralik makul)
      3) son N fix birbirine yakin (oturmus)  -> ortalamasi datum olur
    Ayrica heading (/gps/imu) gecerli birim quaternion olana kadar bekler.
    """

    def __init__(self):
        super().__init__('auto_datum_setter')

        # --- AYARLAR (parametre olarak gecersiz kilinabilir) ---
        # 2 = RTK FIXED (GBAS). Cevre RTK alamiyorsa 1'e (RTK FLOAT) dusurulebilir.
        self.min_fix_status = self.declare_parameter('min_fix_status', NavSatStatus.STATUS_GBAS_FIX).value
        self.samples_needed = self.declare_parameter('samples_needed', 5).value
        self.max_spread_m   = self.declare_parameter('max_spread_m', 1.0).value

        self.gps_topic = '/fix'
        self.imu_topic = '/gps/imu'
        self.service_name = '/datum'

        # --- DEGISKENLER ---
        self.fix_buffer = []            # [(lat, lon, alt), ...] son gecerli RTK fix'ler
        self.latest_imu = None
        self.datum_set_requested = False

        # --- ABONELIKLER ---
        self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)

        # --- SERVIS ISTEMCISI ---
        self.datum_client = self.create_client(SetDatum, self.service_name)

        self.get_logger().info(
            f"Datum icin bekleniyor: status>={self.min_fix_status} (2=RTK FIXED), "
            f"{self.samples_needed} tutarli fix + gecerli heading.")

        while not self.datum_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"'{self.service_name}' servisi bekleniyor... navsat_transform_node acik mi?")

    # ---------------- IMU ----------------
    def imu_callback(self, msg: Imu):
        self.latest_imu = msg

    def _imu_valid(self) -> bool:
        if self.latest_imu is None:
            return False
        q = self.latest_imu.orientation
        n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        return 0.9 < n < 1.1   # birim quaternion -> dual-antenna heading hazir

    # ---------------- GPS ----------------
    def _fix_valid(self, msg: NavSatFix) -> bool:
        if msg.status.status < self.min_fix_status:
            return False
        lat, lon = msg.latitude, msg.longitude
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return False
        if abs(lat) < 1e-3 and abs(lon) < 1e-3:     # Null Island (0,0)
            return False
        if abs(lat) > 90.0 or abs(lon) > 180.0:     # aralik disi
            return False
        return True

    def _consistent(self, buf) -> bool:
        lats = [f[0] for f in buf]
        lons = [f[1] for f in buf]
        lat0 = sum(lats) / len(lats)
        m_per_deg_lat = 111320.0
        m_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))
        dy = (max(lats) - min(lats)) * m_per_deg_lat
        dx = (max(lons) - min(lons)) * m_per_deg_lon
        return math.hypot(dx, dy) <= self.max_spread_m

    def gps_callback(self, msg: NavSatFix):
        if self.datum_set_requested:
            return

        if not self._fix_valid(msg):
            self.fix_buffer.clear()    # tutarlilik penceresini bozma
            self.get_logger().warn(
                f"Uygun fix yok (status={msg.status.status}, gerekli>={self.min_fix_status}). "
                f"Bekleniyor...", throttle_duration_sec=3.0)
            return

        if not self._imu_valid():
            self.get_logger().warn(
                "GPS RTK hazir ama heading (/gps/imu) gecersiz. Bekleniyor...",
                throttle_duration_sec=3.0)
            return

        self.fix_buffer.append((msg.latitude, msg.longitude, msg.altitude))
        if len(self.fix_buffer) < self.samples_needed:
            return

        if not self._consistent(self.fix_buffer):
            self.get_logger().warn(
                "RTK fix'ler henuz oturmadi (yayilim > "
                f"{self.max_spread_m} m). Tampon sifirlandi.", throttle_duration_sec=3.0)
            self.fix_buffer.clear()
            return

        # --- TUM SARTLAR SAGLANDI: ortalama datum ---
        self.datum_set_requested = True
        n = len(self.fix_buffer)
        lat = sum(f[0] for f in self.fix_buffer) / n
        lon = sum(f[1] for f in self.fix_buffer) / n
        alt = sum(f[2] for f in self.fix_buffer) / n
        self.get_logger().info(
            f"RTK-FIXED tutarli ({n} ornek). Datum: lat={lat:.7f} lon={lon:.7f} alt={alt:.2f}")
        self.send_datum_request(lat, lon, alt)

    # ---------------- SERVIS ----------------
    def send_datum_request(self, lat, lon, alt):
        req = SetDatum.Request()
        req.geo_pose.position.latitude = lat
        req.geo_pose.position.longitude = lon
        req.geo_pose.position.altitude = alt
        # Oryantasyon ENU'ya sabit (East=0). Baslangic yonune map'i dondurmuyoruz.
        req.geo_pose.orientation.x = 0.0
        req.geo_pose.orientation.y = 0.0
        req.geo_pose.orientation.z = 0.0
        req.geo_pose.orientation.w = 1.0

        future = self.datum_client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            future.result()
            self.get_logger().info("BASARILI: Datum kuruldu, map (0,0) sabitlendi. Node kapatiliyor...")
            raise SystemExit
        except Exception as e:
            self.get_logger().error(f"Datum servisi hata verdi: {e}")
            self.datum_set_requested = False
            self.fix_buffer.clear()


def main(args=None):
    rclpy.init(args=args)
    node = AutoDatumSetter()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.shutdown()
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
