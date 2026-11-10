from rclpy.node import Node
from geometry_msgs.msg import Twist

from my_robot_commands.model_v2 import RobotCommandModel
from my_robot_commands.view_v2 import RobotDirectionGUI

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
