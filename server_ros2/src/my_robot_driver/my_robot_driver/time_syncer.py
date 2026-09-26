import time
import socket
import struct
import rclpy
from rclpy.node import Node

class ArduinoTimeSyncer(Node):

    def __init__(self):
        super().__init__('arduino_time_syncer')
        
        self.get_logger().info(f"[time_syncer] : launch task")
        
        # --- CONFIGURATION RÉSEAU ---
        # Remplacez par l'IP de votre Arduino Giga et le port configuré pour task_control
        self.arduino_ip = "192.168.1.11" 
        self.arduino_port = 5010 
        
        # Initialisation du socket UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Identifiant de la commande de synchronisation (doit matcher l'Arduino)
        self.COMMAND_ID = 0xEE 
        self.PACKET_FORMAT = '<BQ'  # 1 octet (ID) + 8 octets (uint64 timestamp)
        
        # --- TIMERS ROS2 ---
        # Envoi immédiat du premier paquet au démarrage
        self.send_time_sync()
        
        # Création d'un timer pour répéter la synchronisation toutes les 10.0 secondes
        self.sync_timer = self.create_timer(10.0, self.send_time_sync)
        self.get_logger().info("Nœud de synchronisation temporelle Arduino démarré.")

    def send_time_sync(self):
        try:
            #self.get_logger().info(f"[time_syncer] : send time synchronisation")
            
            # 1. Récupération du temps système actuel du PC (Unix Epoch)
            # time.time() renvoie des secondes sous forme de float. 
            # Multiplié par 1000 pour obtenir des millisecondes absolues.
            current_unix_time_ms = int(time.time() * 1000)
            
            # 2. Encapsulation des données au format binaire binaire Little-Endian
            packet = struct.pack(self.PACKET_FORMAT, self.COMMAND_ID, current_unix_time_ms)
            
            # 3. Envoi via UDP
            self.sock.sendto(packet, (self.arduino_ip, self.arduino_port))
            
            self.get_logger().debug(f"Synchronisation envoyée : {current_unix_time_ms} ms")
            
        except Exception as e:
            self.get_logger().error(f"Échec de l'envoi de la synchronisation UDP : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoTimeSyncer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

