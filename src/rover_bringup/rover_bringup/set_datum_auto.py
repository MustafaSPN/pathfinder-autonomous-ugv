import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from robot_localization.srv import SetDatum
import sys

class AutoDatumSetter(Node):
    def __init__(self):
        super().__init__('auto_datum_setter')

        # --- AYARLAR ---
        self.gps_topic = '/fix'
        self.imu_topic = '/gps/imu'
        self.service_name = '/datum'

        # --- DEĞİŞKENLER ---
        self.latest_fix = None
        self.latest_imu = None
        self.datum_set_requested = False # Tekrar tekrar çağırmayı engellemek için

        # --- ABONELİKLER ---
        # GPS Verisini Dinle
        self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, 10)
        # IMU Verisini Dinle
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)

        # --- SERVİS İSTEMCİSİ ---
        self.datum_client = self.create_client(SetDatum, self.service_name)
        
        self.get_logger().info("Veri bekleniyor... (GPS Fix ve IMU)")

        # Servisin hazır olmasını bekle (Robot Localization çalışıyor mu?)
        while not self.datum_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"'{self.service_name}' servisi bekleniyor... navsat_transform_node açık mı?")

    def gps_callback(self, msg: NavSatFix):
        # Sadece geçerli bir FIX varsa kaydet
        # Status -1 genelde NO_FIX demektir.
        if msg.status.status >= 0:
            self.latest_fix = msg
            self.check_and_set_datum()
        else:
            self.get_logger().warn("GPS verisi geldi ancak FIX yok (Status < 0). Bekleniyor...", throttle_duration_sec=2.0)

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg
        self.check_and_set_datum()

    def check_and_set_datum(self):
        # Eğer zaten işlem başlattıysak veya veriler eksikse çık
        if self.datum_set_requested:
            return
        if self.latest_fix is None or self.latest_imu is None:
            return

        # --- İKİ VERİ DE HAZIR, SERVİSİ ÇAĞIR ---
        self.datum_set_requested = True
        self.get_logger().info(f"Imu= {self.latest_imu.orientation}! Datum ayarlanıyor...")
        self.send_datum_request()

    def send_datum_request(self):
        req = SetDatum.Request()
        
        # 1. Pozisyonu GPS'ten al
        req.geo_pose.position.latitude = self.latest_fix.latitude
        req.geo_pose.position.longitude = self.latest_fix.longitude
        req.geo_pose.position.altitude = self.latest_fix.altitude

        # 2. Oryantasyonu ENU'ya sabitle (ROS2: East=0 yaw)
        # Başlangıç yönüne bağlı kalmasın diye map frame'i döndürmüyoruz.
        req.geo_pose.orientation.x = 0.0
        req.geo_pose.orientation.y = 0.0
        req.geo_pose.orientation.z = 0.0
        req.geo_pose.orientation.w = 1.0

        # 3. Asenkron çağrı yap
        future = self.datum_client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("BAŞARILI: Datum seti yapıldı. Harita (0,0) noktası sabitlendi.")
            self.get_logger().info("Görev tamamlandı, node kapatılıyor...")
            
            # Node'u güvenli şekilde kapat
            raise SystemExit
        except Exception as e:
            self.get_logger().error(f"Datum servisi hata verdi: {e}")
            self.datum_set_requested = False # Hata olduysa tekrar dene

def main(args=None):
    rclpy.init(args=args)
    node = AutoDatumSetter()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        # Başarılı çıkış
        rclpy.shutdown()
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
