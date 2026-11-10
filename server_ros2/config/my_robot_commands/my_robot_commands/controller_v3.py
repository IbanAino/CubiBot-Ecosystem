from rclpy.node import Node
from geometry_msgs.msg import Twist

from my_robot_commands.model_v3 import RobotCommandModel
from my_robot_commands.view_v2 import RobotDirectionGUI
from my_robot_commands.joystick import JoystickInput


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_gui_controller_node')
        
        self.model = RobotCommandModel(max_linear_speed=0.6, max_angular_speed=1.2)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Entrée 1 : Interface graphique (boutons discrets)
        self.view = RobotDirectionGUI(on_direction_click_callback=self.handle_gui_action)
        
        # Entrée 2 : Joystick (axes analogiques continus)
        # On imagine une classe Joystick qui appelle notre méthode en lui passant (axis_y, axis_x)
        self.joystick_input = JoystickInput(on_axis_change_callback=self.handle_joystick_action)
        self.create_timer(0.05, self.joystick_input.poll_events)

    def handle_gui_action(self, direction):
        """Gestion des clics boutons (Tout ou Rien)"""
        linear_val, angular_val = self.model.compute_from_direction(direction)
        self.publish_twist(linear_val, angular_val)

    def handle_joystick_action(self, axis_linear, axis_angular):
        """Gestion des axes analogiques (Valeurs continues entre -1.0 et 1.0)"""
        # Le contrôleur passe directement les floats au modèle
        linear_val, angular_val = self.model.compute_from_axes(axis_linear, axis_angular)
        self.publish_twist(linear_val, angular_val)

    def publish_twist(self, linear, angular):
        """Méthode utilitaire interne pour publier sur le topic ROS2"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

