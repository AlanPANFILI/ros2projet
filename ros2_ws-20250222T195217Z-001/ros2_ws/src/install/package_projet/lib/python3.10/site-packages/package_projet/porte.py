import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DoorController(Node):

    def __init__(self):
        super().__init__('porte')
        self.subscription = self.create_subscription(String, 'commande', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, 'etat_porte', 10)
        self.door_open = False 

    def listener_callback(self, msg):

        if self.door_open:
            new_state = "Porte fermée"
            self.door_open = False
        else:
            new_state = "Porte ouverte"
            self.door_open = True
        
        self.get_logger().info(f'{new_state} en réponse à la commande: {msg.data}')
        self.publish_door_state(new_state)

    def publish_door_state(self, state):
        msg = String()
        msg.data = state
        self.publisher.publish(msg)
        self.get_logger().info(f'État publié: {state}')

def main(args=None):
    rclpy.init(args=args)
    door_controller = DoorController()
    rclpy.spin(door_controller)
    door_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

