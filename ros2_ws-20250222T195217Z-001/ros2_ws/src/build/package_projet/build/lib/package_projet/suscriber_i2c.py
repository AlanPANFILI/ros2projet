import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber_actioneur')

        self.create_subscription(Int32, 'temp', self.listener_callback1, 10)
        self.create_subscription(Int32, 'humid', self.listener_callback2, 10)
        self.create_subscription(Int32, 'qualite', self.listener_callback3, 10)
        
        self.capt1 = 0
        self.capt2 = 0
        self.capt3 = 0


        self.publisher_fenetre = self.create_publisher(String, 'fenetre', 10)
        self.publisher_chauffage = self.create_publisher(String, 'chauffage', 10)

    def listener_callback1(self, msg):
        self.get_logger().info(f'Température reçue: "{msg.data}"')
        self.capt1 = msg.data
        self.evaluate_temperature()

    def listener_callback2(self, msg):
        self.get_logger().info(f'Humidité reçue: "{msg.data}"')
        self.capt2 = msg.data
        
    def listener_callback3(self, msg):
        self.get_logger().info(f'Qualité reçue: "{msg.data}"')
        self.capt3 = msg.data
        self.evaluate_quality()

    def evaluate_quality(self):
        if self.capt3 < 30:
            message = "Ouvre la fenêtre"
        else:
            message = "Ferme la fenêtre"
        

        msg = String()
        msg.data = message
        self.publisher_fenetre.publish(msg)
        self.get_logger().info(f'Publishing on "fenetre": "{msg.data}"')

    def evaluate_temperature(self):
        if self.capt1 < 20:
            message = "Chauffage ON"
        elif self.capt1 > 22:
            message = "Chauffage OFF"
        else:
            return 
            
        msg = String()
        msg.data = message
        self.publisher_chauffage.publish(msg)
        self.get_logger().info(f'Publishing on "chauffage": "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

