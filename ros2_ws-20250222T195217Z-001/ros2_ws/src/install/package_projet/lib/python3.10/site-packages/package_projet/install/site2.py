import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Adapter selon le type de message ROS 2
from flask import Flask, jsonify, render_template
import threading

# Variable globale pour stocker le dernier message reçu
latest_message = "Aucun message reçu"

# ROS 2 Subscriber Node
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('site_flask')
        self.subscription = self.create_subscription(
            String,  # Adapter selon votre message ROS 2
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
    return """
    <html>
    <head>
        <title>ROS 2 Flask Interface</title>
        <script>
            function fetchData() {
                fetch('/data')
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById('ros-message').innerText = data.message;
                    });
            }
            setInterval(fetchData, 1000);  // Mise à jour toutes les secondes
        </script>
    </head>
    <body>
        <h1>Dernier message ROS 2 :</h1>
        <p id="ros-message">Chargement...</p>
    </body>
    </html>
    """

@app.route('/data')
def get_data():
    return jsonify({"message": latest_message})

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

