import socket
import struct
import threading
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.time import Time

from rclpy.executors import MultiThreadedExecutor # 1. Importer l'executor multi-threadétors 


class WifiBridgeOdom(Node):
    """
    Nœud ROS 2 — réception de l'odométrie depuis le robot via UDP WiFi.

    Reçoit des paquets binaires PacketOdometrie sur le port UDP 5007 :

        float x
        float y
        float theta
        float linearVel
        float angularVel

    Total : 5 × 4 = 20 octets

    Publie :
      - nav_msgs/Odometry sur le topic 'odom'
      - la transformation TF odom -> base_link
    """

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    UDP_HOST = '0.0.0.0'
    UDP_PORT = 5007

    # PacketOdometrie :
    #
    # float x
    # float y
    # float theta
    # float linearVel
    # float angularVel
    #
    # Total = 20 octets

    FORMAT_ODOMETRIE = '<Qfffff'
    TAILLE_ODOMETRIE = struct.calcsize(FORMAT_ODOMETRIE)

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------
    
    def __init__(self):
        super().__init__('wifi_bridge_odom')

        self.odom_topic = 'odom'
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'

        self.declare_parameter('host', self.UDP_HOST)
        self.declare_parameter('port', self.UDP_PORT)

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        
        #self.odom_timestamp = 0
        #self.delta_ms_ros2_odom = None

        # 1. AJOUT : Créer un groupe de callbacks réentrant pour le multi-threading
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.callback_group = ReentrantCallbackGroup()

        # 2. MODIFICATION : Associer le publisher au groupe
        self.odom_publisher = self.create_publisher(
            Odometry,
            self.odom_topic,
            10,
            callback_group=self.callback_group # <-- Ajouté
        )

        # 3. MODIFICATION : Associer le Broadcaster TF au groupe (Optionnel mais recommandé)
        # Note : En rclpy, le TransformBroadcaster prend les paramètres d'options s'ils sont configurés,
        # mais pour forcer sa prise en compte dans le groupe, assurez-vous qu'il est lié au nœud principal.
        self.tf_broadcaster = TransformBroadcaster(self)

        # Socket UDP
        self.sock = None
        self.running = True

        # Thread de réception
        self.receiver_thread = threading.Thread(
            target=self._receiver_loop,
            daemon=True
        )

        self.receiver_thread.start()

        self.get_logger().info(
            f'Bridge Odométrie UDP en écoute sur '
            f'{self.host}:{self.port}'
        )
        
    # ------------------------------------------------------------------
    # Réception UDP
    # ------------------------------------------------------------------

    def _receiver_loop(self):
        try:
            self.sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_DGRAM
            )

            self.sock.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_REUSEADDR,
                1
            )

            self.sock.bind((self.host, self.port))

            self.sock.settimeout(1.0)

        except OSError as e:
            self.get_logger().error(
                f'Impossible d ouvrir le socket UDP : {e}'
            )
            self.running = False
            return

        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)

            except socket.timeout:
                continue

            except OSError:
                break

            # Vérification de la taille du paquet
            if len(data) != self.TAILLE_ODOMETRIE:
                self.get_logger().warning(
                    f'Paquet Odométrie incorrect : '
                    f'{len(data)} octets reçus, '
                    f'{self.TAILLE_ODOMETRIE} attendus'
                )
                continue

            decoded = self._decode_packet(data)

            if decoded is not None:
                self._publish_odometry(decoded)

    # ------------------------------------------------------------------
    # Décodage paquet UDP
    # ------------------------------------------------------------------

    def _decode_packet(self, data):
        """
        Décode un paquet UDP brut d'odométrie.

        Retourne :
            (x, y, theta, linearVel, angularVel)

        ou None si le décodage échoue.
        """

        try:
            valeurs = struct.unpack(
                self.FORMAT_ODOMETRIE,
                data
            )

        except struct.error as e:
            self.get_logger().warning(
                f'Erreur décodage Odométrie : {e}'
            )
            return None
            
            
            
        timestamp  = valeurs[0]
        x          = valeurs[1]
        y          = valeurs[2]
        theta      = valeurs[3]
        linear_vel = valeurs[4]
        angular_vel = valeurs[5]

        #self.get_logger().info(
        #    f'Odom : '
        #    f'x={x:.3f} m | '
        #    f'y={y:.3f} m | '
        #    f'theta={theta:.3f} rad | '
        #    f'linearVel={linear_vel:.3f} m/s | '
        #    f'angularVel={angular_vel:.3f} rad/s'
        #)
        
        

        #if self.delta_ms_ros2_odom is None:
        #    now = self.get_clock().now()
        
        #    self.delta_ms_ros2_odom = (
        #        now.nanoseconds // 1_000_000
        #        - timestamp
        #    )

        #odom_timestamp  = timestamp + self.delta_ms_ros2_odom

        return (
            #odom_timestamp,
            timestamp,
            x,
            y,
            theta,
            linear_vel,
            angular_vel
        )

    # ------------------------------------------------------------------
    # Publication ROS2
    # ------------------------------------------------------------------

    def _publish_odometry(self, odometry):
        """
        Publie l'odométrie ROS2 et la transformation TF.
        """
        #now = self.get_clock().now().to_msg()
        
        timestamp, x, y, theta, linear_vel, angular_vel = odometry


        # --------------------------------------------------------------
        # Conversion theta -> quaternion
        # Rotation autour de Z uniquement
        # --------------------------------------------------------------

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        # --------------------------------------------------------------
        # Publication nav_msgs/Odometry
        # --------------------------------------------------------------

        odom_msg = Odometry()

        #odom_msg.header.stamp = now
        odom_msg.header.stamp = Time(nanoseconds=timestamp * 1_000_000).to_msg()
        
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Position
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Vitesse
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel
        
        #self.get_logger().info(
        #    f"[wifi_bridge_odom]🚗{timestamp}" 
        #)
        

        self.odom_publisher.publish(odom_msg)

        # --------------------------------------------------------------
        # Publication TF : odom -> base_link
        # --------------------------------------------------------------

        t = TransformStamped()

        t.header.stamp = Time(nanoseconds=timestamp * 1_000_000).to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    # Arrêt propre
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.running = False

        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass

        super().destroy_node()


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
"""
def main(args=None):

    rclpy.init(args=args)

    node = WifiBridgeOdom()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
"""

def main(args=None):
    rclpy.init(args=args)
    node = WifiBridgeOdom()

    # 2. Remplacer rclpy.spin(node) par un Executor Multi-threadé
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin() # L'executor gère maintenant les publications en parallèle
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()


