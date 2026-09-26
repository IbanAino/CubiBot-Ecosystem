import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')

        # Déclaration des paramètres avec valeurs par défaut
        self.declare_parameter('message_texte', 'Bonjour depuis ROS2')
        self.declare_parameter('frequence', 1.0)

        # Lecture des valeurs (au démarrage)
        message_texte = self.get_parameter('message_texte').value
        frequence = self.get_parameter('frequence').value

        self.publisher_ = self.create_publisher(String, 'topic_demo', 10)
        timer_period = 1.0 / frequence
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.compteur = 0
        self.message_texte = message_texte

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.message_texte}, message n°{self.compteur}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publication : "{msg.data}"')
        self.compteur += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
