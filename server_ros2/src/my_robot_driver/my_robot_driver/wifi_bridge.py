import math
import socket
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class WifiBridge(Node):

    def __init__(self):
        super().__init__('wifi_bridge')
        
        self.odom_topic = 'odom'        # Le nom du canal de communication
        self.odom_frame = 'odom'        # Le nom du repère fixe de départ (standard ROS)

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5007)
        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value

        self.server_sock = None
        self.client_conn = None
        self.client_lock = threading.Lock()
        self.running = True

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.raw_subscription = self.create_subscription(
            String,
            'cmd_raw',
            self.cmd_raw_callback,
            10
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            self.odom_topic,
            10
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # Le serveur TCP tourne dans un thread d'arrière-plan : accept()
        # et recv() sont bloquants, incompatibles avec un timer ROS2 classique.
        self.server_thread = threading.Thread(
            target = self._server_loop,
            daemon=True
        )
        
        self.server_thread.start()
        
        self.get_logger().info(
            f'Bridge Odometrie UDP en écoute sur '
            f'{self.host}:{self.port}'
        )

    # ------------------------------------------------------------------
    # Serveur TCP (thread d'arrière-plan)
    # ------------------------------------------------------------------

    def _server_loop(self):
        try:
            self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_sock.bind((self.host, self.port))
            self.server_sock.listen(1)
            self.server_sock.settimeout(1.0)
            self.get_logger().info(f'Serveur WiFi en écoute sur {self.host}:{self.port}')
        except OSError as e:
            self.get_logger().error(f'Impossible d\'ouvrir le serveur : {e}')
            self.running = False
            return

        while self.running:
            try:
                conn, addr = self.server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self.get_logger().info(f'Robot connecté depuis {addr[0]}:{addr[1]}')
            with self.client_lock:
                self.client_conn = conn

            self._client_loop(conn)

            with self.client_lock:
                self.client_conn = None
            self.get_logger().warning('Robot déconnecté')

    def _client_loop(self, conn):
        conn.settimeout(1.0)
        remainder = ''
        while self.running:
            try:
                chunk = conn.recv(256)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break

            texte = remainder + chunk.decode('ascii', errors='replace')
            lignes = texte.split('\n')
            remainder = lignes[-1]

            for ligne in lignes[:-1]:
                ligne = ligne.strip()
                if ligne.startswith('ODOM,'):
                    self.publish_odometry(ligne)
                    
                    self.get_logger().info(
                        f'Lign eODOM reçue : '
                        f'{ligne}'
                    )
                    

    # ------------------------------------------------------------------
    # Réception -> publication ROS2
    # ------------------------------------------------------------------

    def publish_odometry(self, ligne):
        # Format attendu : ODOM,x,y,theta,vitesse_gauche,vitesse_droite,stall_g,stall_d
        
        print(f">>> DONNEE WIFI RECUE : {ligne}", flush=True) 
        
        champs = ligne.split(',')
        if len(champs) != 8:
            self.get_logger().warning(f'Ligne ODOM malformée ignorée : {ligne}')
            return

        try:
            x = float(champs[1])
            y = float(champs[2])
            theta = float(champs[3])
            v_gauche = float(champs[4])
            v_droite = float(champs[5])
        except ValueError:
            self.get_logger().warning(f'Impossible de parser la ligne ODOM : {ligne}')
            return

        now = self.get_clock().now().to_msg()

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        
        # ⚠️ CORRECTION : Le child_frame de l'odométrie doit correspondre 
        # à la racine du robot (base_link) et non au châssis directement.
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        vitesse_lineaire = (v_gauche + v_droite) / 2.0
        odom_msg.twist.twist.linear.x = vitesse_lineaire

        self.odom_publisher.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # 📊 AJOUT : Bloc de logs pour monitorer l'odométrie dans le terminal (toutes les 1s)
        self.get_logger().info(
            f'🚗 Odométrie active - Pos: [X: {x:.2f}m, Y: {y:.2f}m], Angle: {theta:.2f} rad, Vit: {vitesse_lineaire:.2f} m/s',
            throttle_duration_sec=1.0
        )

    # ------------------------------------------------------------------
    # ROS2 -> envoi vers le robot
    # ------------------------------------------------------------------

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        self._send_command(f'CMD,{linear:.2f},{angular:.2f}')

    def cmd_raw_callback(self, msg):
        commande = msg.data.strip()
        if commande:
            self._send_command(commande)

    def _send_command(self, texte):
        with self.client_lock:
            conn = self.client_conn
        if conn is None:
            self.get_logger().debug('Commande ignorée : aucun robot connecté')
            return
        try:
            conn.sendall(texte.encode('ascii') + b'\r\n')
            self.get_logger().info(f'Envoyé : {texte}')
        except OSError as e:
            self.get_logger().error(f'Envoi échoué : {e}')

    # ------------------------------------------------------------------
    # Arrêt propre
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.running = False
        with self.client_lock:
            if self.client_conn:
                try:
                    self.client_conn.close()
                except OSError:
                    pass
        if self.server_sock:
            try:
                self.server_sock.close()
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WifiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
