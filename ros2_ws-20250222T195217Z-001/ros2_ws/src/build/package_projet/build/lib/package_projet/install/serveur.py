import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class LumiereService(Node):
    def __init__(self):
        super().__init__('lumiere_service')
        self.srv = self.create_service(Trigger, 'toggle_light', self.toggle_light_callback)
        self.get_logger().info('Service "toggle_light" prêt.')

    def toggle_light_callback(self, request, response):
        self.get_logger().info('Requête reçue pour basculer la lumière.')
        response.success = True
        response.message = "Lumière basculée avec succès"
        return response

def main():
    rclpy.init()
    node = LumiereService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
