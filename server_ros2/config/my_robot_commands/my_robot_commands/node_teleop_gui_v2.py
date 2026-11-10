import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
from threading import Thread

# ==========================================
# 1. LE MODÈLE (Logique métier pure - Zéro ROS2, Zéro GUI)
# ==========================================
class RobotCommandModel:
    def __init__(self, max_linear_speed=0.2, max_angular_speed=0.5):
        # Configuration métier du robot
        self._max_linear = max_linear_speed
        self._max_angular = max_angular_speed
        
        # État interne de la commande actuelle
        self.linear_x = 0.0
        self.angular_z = 0.0

    def compute_target_speeds(self, direction):
        """
        Calcule les vitesses cibles selon la direction demandée.
        C'est ici que réside la 'physique' ou les règles métier du robot.
        """
        if direction == "forward":
            self.linear_x = self._max_linear
            self.angular_z = 0.0
        elif direction == "backward":
            self.linear_x = -self._max_linear
            self.angular_z = 0.0
        elif direction == "left":
            self.linear_x = 0.0
            self.angular_z = self._max_angular
        elif direction == "right":
            self.linear_x = 0.0
            self.angular_z = -self._max_angular
        elif direction == "stop":
            self.linear_x = 0.0
            self.angular_z = 0.0
        else:
            raise ValueError(f"Direction inconnue : {direction}")
            
        return self.linear_x, self.angular_z


# ==========================================
# 2. LA VUE (Interface Graphique pure - Passive)
# ==========================================
class RobotDirectionGUI(tk.Tk):
    def __init__(self, on_direction_click_callback):
        super().__init__()
        self.title("Contrôle Robot - MVC Strict")
        self.geometry("300x200")
        
        # La vue reçoit une fonction de rappel, elle ignore qui la traite
        self.trigger_movement = on_direction_click_callback
        
        # Création et configuration des composants graphiques
        btn_up = tk.Button(self, text="▲ Avant", command=lambda: self.trigger_movement("forward"))
        btn_down = tk.Button(self, text="▼ Arrière", command=lambda: self.trigger_movement("backward"))
        btn_left = tk.Button(self, text="◀ Gauche", command=lambda: self.trigger_movement("left"))
        btn_right = tk.Button(self, text="▶ Droite", command=lambda: self.trigger_movement("right"))
        btn_stop = tk.Button(self, text="■ STOP", command=lambda: self.trigger_movement("stop"), bg="red", fg="white")
        
        # Placement strict via la grille
        btn_up.grid(row=0, column=1, pady=5)
        btn_left.grid(row=1, column=0, padx=5)
        btn_stop.grid(row=1, column=1, padx=5)
        btn_right.grid(row=1, column=2, padx=5)
        btn_down.grid(row=2, column=1, pady=5)


# ==========================================
# 3. LE CONTRÔLEUR (Orchestrateur & Nœud ROS2)
# ==========================================
class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_gui_controller_node')
        
        # Initialisation du Modèle (Paramétrable et indépendant)
        self.model = RobotCommandModel(max_linear_speed=0.6, max_angular_speed=1.2)
        
        # Configuration de l'infrastructure ROS2
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Nœud MVC configuré. Publication sur /cmd_vel")
        
        # Initialisation de la Vue en lui injectant uniquement notre méthode de gestion
        self.view = RobotDirectionGUI(on_direction_click_callback=self.handle_ui_action)

    def handle_ui_action(self, direction):
        """Intercepte l'action de la vue, interroge le modèle et pilote ROS2."""
        self.get_logger().info(f"Événement GUI intercepté : {direction}")
        
        # 1. On demande au modèle de calculer l'état ou la commande métier
        linear_val, angular_val = self.model.compute_target_speeds(direction)
        
        # 2. On formate le message ROS2 avec les données issues du modèle
        msg = Twist()
        msg.linear.x = linear_val
        msg.angular.z = angular_val
        
        # 3. On publie sur le middleware
        self.publisher_.publish(msg)


# ==========================================
# POINT D'ENTRÉE DU SYSTÈME
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    
    controller_node = RobotControllerNode()
    
    # Threading des entrées/sorties ROS2 pour ne pas bloquer l'UI principale
    ros_thread = Thread(target=rclpy.spin, args=(controller_node,), daemon=True)
    ros_thread.start()
    
    try:
        controller_node.view.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

