import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleServiceClient(Node):

    def __init__(self):
        super().__init__('simple_service_client')
        self.client = self.create_client(AddTwoInts, 'addition')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('En attente du serveur...')

    def envoyer_requete(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceClient()

    a = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 4

    resultat = node.envoyer_requete(a, b)
    node.get_logger().info(f'Résultat : {a} + {b} = {resultat.sum}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
