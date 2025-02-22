import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import random
import time

class LumiereClient(Node):
    def __init__(self):
        super().__init__('lumiere_client')
        self.client = self.create_client(Trigger, 'toggle_light')  # Changer le service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attente du service...')
        self.request = Trigger.Request()

    def send_request(self):
        delay = random.uniform(1, 5)  # Délai aléatoire entre 1 et 5 secondes
        self.get_logger().info(f'Attente de {delay:.2f} secondes avant d\'envoyer la requête...')
        time.sleep(delay)

        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        print(f'Response: {future.result().message}')

def main():
    rclpy.init()
    client = LumiereClient()
    client.send_request()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

