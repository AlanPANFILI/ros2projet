import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class LampService(Node):
    def __init__(self):
        super().__init__('lamp_service')
        self.lampe_allumee = False
        self.srv = self.create_service(Trigger, 'allumer_lampe', self.allumer_lampe_callback)
        self.get_logger().info('Service "allumer_lampe" prêt.')

    def allumer_lampe_callback(self, request, response):
        self.lampe_allumee = not self.lampe_allumee

        if self.lampe_allumee:
            response.success = True
            response.message = "Lampe allumée"
        else:
            response.success = True
            response.message = "Lampe éteinte"

        self.get_logger().info(f"État de la lampe : {'ON' if self.lampe_allumee else 'OFF'}")
        return response

def main():
    rclpy.init()
    node = LampService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

