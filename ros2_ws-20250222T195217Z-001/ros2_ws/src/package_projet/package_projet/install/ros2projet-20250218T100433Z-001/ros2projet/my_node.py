import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer= self.create_timer(0.5, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
        
def main(arg=None):
     rclpy.init(args=arg)
     node = MyNode()
     rclpy.spin(node)
     node.destroy_node()
     rclpy.shutdown()
     
