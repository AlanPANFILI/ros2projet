import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher_presence')
        self.publisher_ = self.create_publisher(Int32, 'presence', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.capt = 1

    def timer_callback(self):
        msg = Int32()
        if (self.capt == 1 ) :
            msg.data = 1
        else :
            msg.data = 0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)
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
