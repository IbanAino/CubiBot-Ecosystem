from rclpy.node import Node
from std_msgs.msg import String


class CommandModel(Node):
    """
    Modèle (MVC) : responsable uniquement de la publication ROS2.

    Le modèle publie une commande lorsqu'il la reçoit de l'IHM.
    La répétition périodique des commandes est gérée par
    wifi_bridge_cmd.py.
    """

    def __init__(self):
        super().__init__('teleop_gui')

        self.publisher_ = self.create_publisher(
            String,
            'cmd_raw',
            10
        )

    def send_command(self, texte: str) -> None:
        """
        Publie une commande unique sur /cmd_raw.
        """

        msg = String()
        msg.data = texte

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Commande envoyée : {texte}'
        )


