import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import time

class EmptyMapPublisher(Node):

    def __init__(self):
        super().__init__('empty_map_publisher')
        
        # Publisher oluştur (QoS: Transient Local - Sonradan gelenler de görsün diye)
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos_policy)
        
        # Harita özelliklerini ayarla
        self.map_resolution = 0.5  # Her piksel 50cm (0.5 metre)
        self.map_size_m = 1000.0    # 1000 metre genişlik
        
        # Grid Boyutu (Piksel cinsinden)
        self.grid_width = int(self.map_size_m / self.map_resolution)  # 1000 piksel
        self.grid_height = int(self.map_size_m / self.map_resolution) # 1000 piksel

        timer_period = 2.0  # Her 2 saniyede bir yayınla (Map statik olduğu için sık basmaya gerek yok)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Bos harita hazirlandi: {self.map_size_m}x{self.map_size_m}m')

    def timer_callback(self):
        msg = OccupancyGrid()
        
        # Header Bilgileri
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Harita Meta-Verisi
        msg.info.resolution = self.map_resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        
        # Haritanın Başlangıç Noktası (Origin)
        # Robotu (0,0) noktasında tutmak için haritayı ortalıyoruz.
        # X: -50m, Y: -50m noktasından başlar.
        msg.info.origin.position.x = -(self.map_size_m / 2.0)
        msg.info.origin.position.y = -(self.map_size_m / 2.0)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0 
        msg.info.origin.orientation.w = 1.0 

        # Harita Verisi (0: Boş, 100: Dolu, -1: Bilinmiyor)
        # 100x100m alanı "0" (Boş) ile dolduruyoruz.
        # Python listesi oluşturuyoruz: [0, 0, 0, ...]
        msg.data = [0] * (self.grid_width * self.grid_height)

        self.publisher_.publish(msg)
        # self.get_logger().info('Harita yayinlaniyor...')

def main(args=None):
    rclpy.init(args=args)
    node = EmptyMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()