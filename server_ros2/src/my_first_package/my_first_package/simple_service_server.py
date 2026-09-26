import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleServiceServer(Node):

    def __init__(self):
        super().__init__('simple_service_server')
        self.srv = self.create_service(
            AddTwoInts,
            'addition',
            self.callback_addition
        )
        self.get_logger().info('Serveur de service prêt sur /addition')

    def callback_addition(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Requête reçue : {request.a} + {request.b} = {response.sum}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
