import rclpy
import random
import time
from rclpy.node import Node
from std_srvs.srv import Trigger

class LampClient(Node):
    def __init__(self):
        super().__init__('lamp_client')
        self.client = self.create_client(Trigger, 'allumer_lampe')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('En attente du service "allumer_lampe"...')

        self.get_logger().info('Service trouvé, envoi des requêtes en boucle...')
        self.send_requests_loop()

    def send_requests_loop(self):
        while rclpy.ok():
            self.send_request()
            delay = random.uniform(1, 10)  # Délai aléatoire entre 1 et 10 secondes
            self.get_logger().info(f"Prochaine requête dans {delay:.2f} secondes...")
            time.sleep(delay)

    def send_request(self):
        req = Trigger.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Réponse du serveur : {response.message}")
        except Exception as e:
            self.get_logger().error(f"Erreur du service : {e}")

def main():
    rclpy.init()
    node = LampClient()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

