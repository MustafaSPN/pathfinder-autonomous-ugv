import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from array import array

class EmptyMapPublisher(Node):

    def __init__(self):
        super().__init__('empty_map_publisher')

        # Publisher (QoS: Transient Local - latched, sonradan gelenler de görür)
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos_policy)

        # Harita özellikleri (sadece görselleştirme için; nav2 global costmap
        # artık rolling_window kullanıyor, bu haritaya bağlı değil).
        # 1000m alan KORUNDU; çözünürlük kabalaştırılarak hücre sayısı düşürüldü.
        self.map_resolution = 1.0    # Her piksel 1m (smooth rota icin 2.0'den inceltildi)
        self.map_size_m = 1000.0     # 1000x1000m -> 1000x1000 = 1M hucre (bir kez uretilir)

        self.grid_width = int(self.map_size_m / self.map_resolution)
        self.grid_height = int(self.map_size_m / self.map_resolution)

        # Mesajı BİR KEZ oluştur ve önbelleğe al (her döngüde 40K liste üretmeyiz).
        self.msg = self._build_map()

        # Latched olduğu için bir kez basmak yeter; yine de geç bağlanan
        # araçlar için seyrek bir heartbeat (önbellekteki mesajı tekrar basar, CPU ~0).
        self.publisher_.publish(self.msg)
        self.timer = self.create_timer(5.0, self._heartbeat)

        self.get_logger().info(
            f'Bos harita hazir: {self.map_size_m}x{self.map_size_m}m '
            f'({self.grid_width}x{self.grid_height} hucre)')

    def _build_map(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.resolution = self.map_resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height

        # Robotu (0,0) merkezinde tutmak için haritayı ortala.
        msg.info.origin.position.x = -(self.map_size_m / 2.0)
        msg.info.origin.position.y = -(self.map_size_m / 2.0)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Tüm alan boş (0). bytes(n) -> n adet sıfır, array('b', ...) hızlı doldurur.
        n = self.grid_width * self.grid_height
        msg.data = array('b', bytes(n))
        return msg

    def _heartbeat(self):
        # Önbellekteki mesajı yeniden basar; yeni liste üretmez.
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmptyMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
