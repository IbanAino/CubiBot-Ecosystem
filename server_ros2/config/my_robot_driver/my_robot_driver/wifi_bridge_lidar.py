import socket
import struct
import threading
import math

import std_msgs.msg

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from rclpy.time import Time


class WifiBridgeLidar(Node):

    UDP_HOST = '0.0.0.0'
    UDP_PORT = 5006
    POINTS_PAR_TRAME = 12
    
    FORMAT_LIDAR = "<QHHH" + "HB" * POINTS_PAR_TRAME
    
    SEUIL_INTENSITE_MIN = 100.0
    
    RANGE_MIN = 0.1       
    RANGE_MAX = 20.0      

    def __init__(self):
        super().__init__('wifi_bridge_lidar')
        
        self.lidar_topic = 'scan'
        self.lidar_frame = 'laser' 
        self.declare_parameter('host', self.UDP_HOST)
        self.declare_parameter('port', self.UDP_PORT)
        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value

        self.callback_group = ReentrantCallbackGroup()

        self.scan_publisher = self.create_publisher(
            LaserScan,
            self.lidar_topic,
            10,
            callback_group=self.callback_group
        )

        # Buffers pour accumuler une révolution complète (360°)
        self.accumulated_ranges = [float('inf')] * 360
        self.accumulated_intensities = [0.0] * 360
        self.last_start_angle = -1.0
        
        self.lidar_timestamp = 0
        #self.delta_ms_ros2_lidar = None

        self.sock = None
        self.running = True

        self.receiver_thread = threading.Thread(
            target=self._receiver_loop,
            daemon=True
        )
        self.receiver_thread.start()

        self.get_logger().info(f'Bridge Lidar UDP en écoute sur {self.host}:{self.port}')


    def _receiver_loop(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(1.0)
        except OSError as e:
            self.get_logger().error(f'Impossible d ouvrir le socket UDP : {e}')
            self.running = False
            return

        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:    
                break
              
            if data:
                resultat = self._decode_packet(data)
                if resultat is not None:
                    timestamp, speed, points, startAngle, endAngle = resultat

                    #if self.delta_ms_ros2_lidar is None:
                    #    now = self.get_clock().now()
                        
                    #    self.delta_ms_ros2_lidar = (
                    #        now.nanoseconds // 1_000_000
                    #        - timestamp
                    #    )
                        
                    #self.lidar_timestamp  = timestamp + self.delta_ms_ros2_lidar
                    self.lidar_timestamp  = timestamp 

                    self._process_and_accumulate(points, startAngle, endAngle)
    
    
    def _decode_packet(self, data):        
        try:
            valeurs = struct.unpack(self.FORMAT_LIDAR, data)
        except struct.error as e:
            self.get_logger().warning(f'Erreur décodage Lidar (Reçu {len(data)} octets) : {e}')
            return None
        
        timestamp  = valeurs[0]
        speed      = valeurs[1]
        startAngle = valeurs[2]
        endAngle   = valeurs[3]
        
        # 2. Extraction magique des points en séparent l'en-tête du reste
        valeurs_points = valeurs[4:] # Contient [dist0, int0, dist1, int1, ...]
        
        points = []
        
        # Chaîne de texte pour accumuler le debug de la trame
        debug_points_str = ""
        
        for i in range(self.POINTS_PAR_TRAME):
            # Indexation dans le tuple à plat : 4, 6, 8... pour les H / 5, 7, 9... pour les B
            base = 4 + (i * 2)
            #distance = valeurs[base]
            #intensity = valeurs[base + 1]
            
            distance  = valeurs_points[i * 2]
            intensity = valeurs_points[(i * 2) + 1]
            
            points.append((distance, intensity))
            
            # On ajoute chaque point au format "[index] Dist: Xmm (Int: Y)"
            #debug_points_str += f"[{i}] {distance}mm (i:{intensity}) | "
            debug_points_str += f"{distance}-({intensity}) | "

        # --- AFFICHAGE DU LOG DÉTAILLÉ DE LA TRAME ---
        #self.get_logger().info(
        #    #f"📥 Trame {timestamp} | Angles: {startAngle/100.0}° -> {endAngle/100.0}° | "
        #    f"📥{timestamp}-{speed}-{startAngle}-{endAngle}" 
        #    f" -> {debug_points_str}"
        #)
        # ---------------------------------------------
        
        return timestamp, speed, points, startAngle, endAngle


    def _process_and_accumulate(self, point_list, startAngle, endAngle):
        nb_points = len(point_list)
        start_deg = startAngle / 100.0
        end_deg = endAngle / 100.0

        # Gestion du passage par le zéro
        if end_deg < start_deg:
            end_deg += 360.0

        # Détection d'un nouveau tour complet (si l'angle recule brusquement)
        # Saut de plus de 180° seulement — robuste au bruit angulaire
        if self.last_start_angle >= 0.0 and start_deg < self.last_start_angle - 180.0:
            self._publish_full_scan()
            self.accumulated_ranges      = [float('inf')] * 360
            self.accumulated_intensities = [0.0]          * 360

        self.last_start_angle = start_deg

        # Interpolation et placement des 12 points dans les bonnes cases du tableau de 360°
        for i, (distance_mm, intensity) in enumerate(point_list):
            # Interpolation de l'angle en degrés pour ce point précis
            angle_point_deg = start_deg + (end_deg - start_deg) * i / (nb_points - 1)
            angle_index = int(angle_point_deg) % 360
            #angle_index = int((-angle_point_deg) % 360)

            # FILTRE D'INTENSITÉ : Si le point n'est pas fiable, on l'efface
            if float(intensity) < self.SEUIL_INTENSITE_MIN:
                self.accumulated_ranges[angle_index] = float('inf')
                self.accumulated_intensities[angle_index] = 0.0
                continue  # On passe immédiatement au point suivant

            # Conversion en mètres
            distance_m = distance_mm / 1000.0
            
            if self.RANGE_MIN <= distance_m <= self.RANGE_MAX:
                # Garde la mesure la plus courte si conflit sur la même case
                if distance_m < self.accumulated_ranges[angle_index]:
                    self.accumulated_ranges[angle_index]      = distance_m
                    self.accumulated_intensities[angle_index] = float(intensity)


    def _publish_full_scan(self):
        """
        Publie le tour complet de 360 degrés accumulé d'un seul coup.
        """
        
        scan = LaserScan()
        #scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.stamp = Time(nanoseconds=self.lidar_timestamp * 1_000_000).to_msg()
        scan.header.frame_id = self.lidar_frame

        # Configuration pour un scan complet à 360°
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = (2.0 * math.pi) / 360.0

        scan.time_increment = 0.0
        scan.scan_time = 0.22  # Environ 10Hz de rotation
        scan.range_min = self.RANGE_MIN
        scan.range_max = self.RANGE_MAX

        scan.ranges = self.accumulated_ranges
        scan.intensities = self.accumulated_intensities
        
        #self.get_logger().info(
        #    #f"📥 Trame {timestamp} | Angles: {startAngle/100.0}° -> {endAngle/100.0}° | "
        #f"📥{self.lidar_timestamp}" 
        #    f" -> {debug_points_str}"
        #)
        
        #self.get_logger().info(
        #    f"[wifi_bridge_lidar]🔆{self.lidar_timestamp}" 
        #)

        self.scan_publisher.publish(scan)


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = WifiBridgeLidar()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

