import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import random

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher_i2c')
        self.publisher_temp_ = self.create_publisher(Int32, 'temp', 10)
        self.publisher_humid_ = self.create_publisher(Int32, 'humid', 10)
        self.publisher_qualite_ = self.create_publisher(Int32, 'qualite', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg_temp = Int32()
        msg_temp.data = random.randint(20, 30)  # Température aléatoire entre 20 et 30°C
        
        msg_humid = Int32()
        msg_humid.data = random.randint(50, 100)  # Humidité aléatoire entre 50 et 100%
        
        msg_qualite = Int32()
        msg_qualite.data = random.randint(0, 50)  # Qualité aléatoire entre 0 et 50
        
        self.publisher_temp_.publish(msg_temp)
        self.get_logger().info(f'Publishing Temp: {msg_temp.data}')
        
        self.publisher_humid_.publish(msg_humid)
        self.get_logger().info(f'Publishing Humid: {msg_humid.data}')
        
        self.publisher_qualite_.publish(msg_qualite)
        self.get_logger().info(f'Publishing Qualite: {msg_qualite.data}')


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

