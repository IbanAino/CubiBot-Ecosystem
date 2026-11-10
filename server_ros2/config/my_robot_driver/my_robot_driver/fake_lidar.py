import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeLidar(Node):
    """
    Nœud ROS 2 — simulation d'un lidar 2D.

    Publie un sensor_msgs/LaserScan avec des points uniformément
    répartis sur un cercle complet (0° → 360°) à une distance fixe
    de 50 cm autour du lidar.

    Paramètres ROS 2 configurables :
        frame_id      : repère du lidar (défaut : 'laser_frame')
        radius_m      : rayon du cercle simulé en mètres (défaut : 0.5)
        num_points    : nombre de points par scan (défaut : 360)
        frequency_hz  : fréquence de publication en Hz (défaut : 10.0)
    """

    def __init__(self):
        super().__init__('fake_lidar')

        self.declare_parameter('frame_id',     'laser')
        self.declare_parameter('radius_m',     0.5)
        self.declare_parameter('num_points',   360)
        self.declare_parameter('frequency_hz', 10.0)

        self.frame_id    = self.get_parameter('frame_id').value
        self.radius_m    = self.get_parameter('radius_m').value
        self.num_points  = self.get_parameter('num_points').value
        self.frequency_hz = self.get_parameter('frequency_hz').value

        self.publisher = self.create_publisher(LaserScan, 'scan', 10)

        period = 1.0 / self.frequency_hz
        self.timer = self.create_timer(period, self._publish_scan)

        self.get_logger().info(
            f'Fake Lidar démarré — '
            f'{self.num_points} points à {self.radius_m}m '
            f'@ {self.frequency_hz}Hz '
            f'sur le repère "{self.frame_id}"'
        )

    def _publish_scan(self):
        #now = self.get_clock().now().to_msg()

        scan = LaserScan()
        #scan.header.stamp    = now
        scan.header.frame_id = self.frame_id

        scan.angle_min       = 0.0                        # 0°
        scan.angle_max       = 2.0 * math.pi              # 360°
        scan.angle_increment = (
            2.0 * math.pi / self.num_points
        )

        scan.time_increment = 0.0
        scan.scan_time      = 1.0 / self.frequency_hz

        scan.range_min = 0.02
        scan.range_max = 12.0

        # Tous les points sont à la même distance (cercle parfait)
        #scan.ranges      = [self.radius_m] * self.num_points
        
        ranges = [5.0] * self.num_points

        for i in range(80, 120):
            ranges[i] = 2.0

        scan.ranges = ranges        
        
        
        scan.intensities = [100.0]         * self.num_points
        
        #self.get_logger().info(
        #    f"[SCAN] {len(scan.ranges)} points | "
        #    f"Angles: {math.degrees(scan.angle_min):.1f}° à {math.degrees(scan.angle_max):.1f}° | "
        #    f"Freq: {1.0 / (scan.scan_time if scan.scan_time > 0 else 0.1):.1f} Hz"
        #    f"Échantillon distances (m): {[round(r, 2) for r in scan.ranges[:5]]}"
        #    f"Échantillon angles (deg): {[round(math.degrees(scan.angle_min + (i * scan.angle_increment)), 1) for i in range(len(scan.ranges[:5]))]}"
        #)
        
        scan.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
