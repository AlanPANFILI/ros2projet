import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node

from std_msgs.msg import Int32
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('suscriber_base')
        self.subscription = self.create_subscription(Int32,'temp',self.listener_callback1,10)
        self.subscription = self.create_subscription(Int32,'humid',self.listener_callback2,10)
        self.subscription = self.create_subscription(Int32,'qualite',self.listener_callback3,10)
        self.subscription  
        self.capt1=0
        self.capt2=0
        

    def listener_callback1(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.data)
        self.capt1=msg.data
#        somme(self)
        
    def listener_callback2(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.data)
        self.capt2=msg.data
#        somme(self)
        
        
    def listener_callback3(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.data)
        self.capt2=msg.data
#        somme(self)


def somme(self) :
    k = self.get_parameter('my_parameter').value
    self.somme=self.capt1+k*self.capt2
    print(f"la somme est: {self.somme}")





def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
       

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
