import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import serial


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        self.serial_conn = None
        self.connected = False
        self._try_connect()

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        self.raw_subscription = self.create_subscription(
            String, 'cmd_raw', self.cmd_raw_callback, 10
        )

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer qui vide le port série ET traite les lignes ODOM (actif seulement si connecté)
        self.read_timer = self.create_timer(0.02, self.read_serial_callback)

        # Timer qui retente une connexion périodiquement (actif seulement si déconnecté)
        self.reconnect_timer = self.create_timer(2.0, self.reconnect_callback)

    def _try_connect(self):
        """Tente d'ouvrir le port série. Ne lève jamais d'exception fatale."""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1, write_timeout=1)
            self.connected = True
            self.get_logger().info(f'Connecté au robot sur {self.port} à {self.baudrate} bauds')
        except (serial.SerialException, OSError) as e:
            self.connected = False
            self.get_logger().warning(f'Port série {self.port} indisponible : {e}')

    def _handle_disconnection(self, e):
        """Appelé dès qu'une erreur série survient pendant une lecture/écriture."""
        self.get_logger().error(f'Connexion série perdue : {e}')
        self.connected = False
        if self.serial_conn is not None and self.serial_conn.is_open:
            self.serial_conn.close()

    def reconnect_callback(self):
        """Ne fait quelque chose que si on est actuellement déconnecté."""
        if self.connected:
            return
        self._try_connect()

    def cmd_vel_callback(self, msg):
        if not self.connected:
            self.get_logger().debug('Commande cmd_vel ignorée : robot déconnecté')
            return
        linear = msg.linear.x
        angular = msg.angular.z
        commande = f'CMD,{linear:.2f},{angular:.2f}\n'
        self._write_serial(commande)

    def cmd_raw_callback(self, msg):
        if not self.connected:
            self.get_logger().debug('Commande cmd_raw ignorée : robot déconnecté')
            return
        commande = msg.data.strip()
        if not commande:
            return
        self._write_serial(commande + '\n')

    def _write_serial(self, commande):
        try:
            self.serial_conn.write(commande.encode('utf-8'))
            self.get_logger().info(f'Envoyé : {commande.strip()}')
        except (serial.SerialException, OSError) as e:
            self._handle_disconnection(e)

    def read_serial_callback(self):
        if not self.connected:
            return
        try:
            while self.serial_conn.in_waiting > 0:
                ligne = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if ligne.startswith('ODOM,'):
                    self.publish_odometry(ligne)
        except (serial.SerialException, OSError) as e:
            self._handle_disconnection(e)

    def publish_odometry(self, ligne):
        # Format attendu : ODOM,x,y,theta,vitesse_gauche,vitesse_droite,stall_g,stall_d
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
        odom_msg.child_frame_id = 'chassis'

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
        t.header.frame_id = 'odom'
        t.child_frame_id = 'chassis'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        if self.serial_conn is not None and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
