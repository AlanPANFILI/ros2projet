import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher_i2c')
        self.publisher_temp_ = self.create_publisher(Int32, 'temp', 10)
        self.publisher_humid_ = self.create_publisher(Int32, 'humid', 10)
        self.publisher_qualite_ = self.create_publisher(Int32, 'qualite', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.temp = 25
        self.humid = 75
        self.qualite = 32

    def timer_callback(self):
        msg_temp= Int32()
        msg_temp.data = self.temp
        msg_humid= Int32()
        msg_humid.data = self.humid
        msg_qualite= Int32()
        msg_qualite.data = self.qualite
        
        
        self.publisher_temp_.publish(msg_temp)
        self.get_logger().info('Publishing: "%d"' % msg_temp.data)
        self.publisher_humid_.publish(msg_humid)
        self.get_logger().info('Publishing: "%d"' % msg_humid.data)
        self.publisher_qualite_.publish(msg_qualite)
        self.get_logger().info('Publishing: "%d"' % msg_qualite.data)
        
        
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
