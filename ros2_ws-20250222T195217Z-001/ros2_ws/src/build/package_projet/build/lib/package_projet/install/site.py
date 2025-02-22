import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Changez le type selon votre besoin
from flask import Flask, render_template
import threading

# Variable globale pour stocker le dernier message ROS 2
latest_message = "Aucun message reçu"

# ROS 2 Subscriber Node
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('site_flask')
        self.subscription = self.create_subscription(
            String,  # Changez le type si besoin
            'topic',  # Remplacez par le nom de votre topic
            self.listener_callback,
            10)
        self.subscription  # Empêche la suppression prématurée

    def listener_callback(self, msg):
        global latest_message
        latest_message = msg.data
        self.get_logger().info(f"Message reçu: {msg.data}")

# Flask App
app = Flask(__name__)

@app.route('/')
def index():
    return f"""
    <html>
    <head><title>ROS 2 Flask Interface</title></head>
    <body>
        <h1>Dernier message ROS 2 :</h1>
        <p>{latest_message}</p>
        <button onclick="location.reload()">Actualiser</button>
    </body>
    </html>
    """

# Démarrage de ROS 2 en parallèle
def start_ros2():
    rclpy.init()
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# Exécuter Flask et ROS 2 simultanément
if __name__ == '__main__':
    ros_thread = threading.Thread(target=start_ros2, daemon=True)
    ros_thread.start()
    app.run(host='0.0.0.0', port=5000, debug=True)

