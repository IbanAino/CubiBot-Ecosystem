import socket
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import String





class WifiBridgeCmd(Node):
    """
    Nœud ROS 2 — Pont de communication UDP binaire.

    Écoute /cmd_raw (généré par l'IHM) et transmet
    les commandes au robot via UDP.

    Les commandes de vitesse sont répétées périodiquement
    afin de permettre au robot de détecter une perte de liaison.
    """

    DEFAULT_ROBOT_IP = '192.168.1.11'
    DEFAULT_ROBOT_PORT = 5010

    CMD_VELOCITY = 0x01
    CMD_STOP = 0x02
    CMD_RESET = 0x03

    COMMAND_FREQUENCY = 20.0

    def __init__(self):
        super().__init__('wifi_bridge_cmd')

        self.declare_parameter('robot_ip', self.DEFAULT_ROBOT_IP)
        self.declare_parameter('robot_port', self.DEFAULT_ROBOT_PORT)

        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.command_active = False
        self.msg_type = ""
        #self.struct_pack: bytes = b""
        
        self.struct_pack = struct.pack(
            '<B',
            self.CMD_STOP
        )

        self.create_subscription(
            String,
            'cmd_raw',
            self._cmd_raw_callback,
            10
        )

        self.command_timer = self.create_timer(
            1.0 / self.COMMAND_FREQUENCY,
            self._loop_send_current_command
        )

        self.get_logger().info(
            f'Bridge Commandes connecté à /cmd_raw '
            f'-> UDP {self.robot_ip}:{self.robot_port} '
            f'({self.COMMAND_FREQUENCY:.1f} Hz)'
        )


    """
    Fonction pour gérer l'envoi de packets en UDP
    """
    def _send_packet(self, packet):
        """
        Envoie un paquet UDP sans laisser une erreur réseau
        tuer le nœud ROS2.
        """

        try:
            self.sock.sendto(
                packet,
                (self.robot_ip, self.robot_port)
            )

            return True

        except OSError as e:
            self.get_logger().warning(
                f'Erreur réseau UDP vers '
                f'{self.robot_ip}:{self.robot_port} : {e}'
            )

            return False

    """
    Fonction de callback
    appellée lors de la publication d'un message de commande sur le topic cmd_raw
    """
    def _cmd_raw_callback(self, msg):
        texte = msg.data.strip()

        if texte.startswith('CMD,'):
            try:
                parts = texte.split(',')

                if len(parts) != 3:
                    raise ValueError(
                        'Format attendu : CMD,linear,angular'
                    )

                linear = float(parts[1])
                angular = float(parts[2])
                


                self.struct_pack = struct.pack(
                    '<Bff',
                    self.CMD_VELOCITY,
                    linear,
                    angular
                )






                #self.current_linear = linear
                #self.current_angular = angular
                #self.command_active = True

                self.get_logger().debug(
                    f'Nouvelle commande : '
                    f'lin={linear:.3f}, ang={angular:.3f}'
                )

            except (ValueError, IndexError) as e:
                self.get_logger().error(
                    f'Commande CMD invalide ({texte}) : {e}'
                )

        elif texte == 'STOP':
            self.command_active = False

            self.struct_pack = struct.pack(
                '<B',
                self.CMD_STOP
            )

            #self._send_packet(packet)

            self.get_logger().debug(
                'Transmis à l’Arduino : STOP'
            )

        elif texte == 'RESET':
            self.command_active = False

            self.struct_pack = struct.pack(
                '<B',
                self.CMD_RESET
            )

            #self._send_packet(packet)

            self.get_logger().debug(
                'Transmis à l’Arduino : RESET'
            )

        else:
            self.get_logger().warning(
                f'Commande /cmd_raw inconnue : "{texte}"'
            )


    """
    Boucle exécutée à fréquence régulière
    fréquence : 20Hz
    """
    def _loop_send_current_command(self):

        #if not self.command_active:
        #    return

        #packet = struct.pack(
        #    '<Bff',
        #    self.CMD_VELOCITY,
        #    self.current_linear,
        #    self.current_angular
        #)

        self._send_packet(self.struct_pack )


    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = WifiBridgeCmd()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


