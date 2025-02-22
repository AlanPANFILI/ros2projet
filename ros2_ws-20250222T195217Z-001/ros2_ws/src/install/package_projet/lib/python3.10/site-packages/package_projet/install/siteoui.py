import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32, String
from flask import Flask, jsonify, request
import threading
import sqlite3

# Variables pour stocker les dernières valeurs
latest_temp = "N/A"
latest_humid = "N/A"
latest_qualite = "N/A"

# ROS 2 Subscriber Node
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('site_flask_int')
        self.create_subscription(Int32, 'temp', self.temp_callback, 10)
        self.create_subscription(Int32, 'humid', self.humid_callback, 10)
        self.create_subscription(Int32, 'qualite', self.qualite_callback, 10)

    def temp_callback(self, msg):
        global latest_temp
        latest_temp = msg.data
        self.get_logger().info(f"Température reçue: {msg.data}")

    def humid_callback(self, msg):
        global latest_humid
        latest_humid = msg.data
        self.get_logger().info(f"Humidité reçue: {msg.data}")

    def qualite_callback(self, msg):
        global latest_qualite
        latest_qualite = msg.data
        self.get_logger().info(f"Qualité de l'air reçue: {msg.data}")

# ROS 2 Publisher Node
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('site_flask_out')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

    def send_message(self, msg_text):
        msg = String()
        msg.data = msg_text
        self.publisher_.publish(msg)
        self.get_logger().info(f"Message envoyé: {msg_text}")

# Initialisation du nœud ROS 2
rclpy.init()
publisher_node = MinimalPublisher()
subscriber_node = MinimalSubscriber()

# Flask App
app = Flask(__name__)

@app.route('/')
def index():
    return """
    <html>
    <head>
        <title>ROS 2 Flask Interface</title>
        <script>
            function fetchLiveData() {
                fetch('/live_data')
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById('live-temp').innerText = data.temp;
                        document.getElementById('live-humid').innerText = data.humid;
                        document.getElementById('live-qualite').innerText = data.qualite;
                    });
            }
            function fetchStoredData() {
                fetch('/data')
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById('stored-temp').innerText = data.temp;
                        document.getElementById('stored-humid').innerText = data.humid;
                        document.getElementById('stored-qualite').innerText = data.qualite;
                    });
            }
            function sendCommand() {
                fetch('/send', { method: 'POST' })
                    .then(response => response.json())
            }
            setInterval(fetchLiveData, 1000);
            setInterval(fetchStoredData, 5000);
        </script>
    </head>
    <body>
        <h1>Valeurs en temps réel :</h1>
        <p>Température: <span id="live-temp">Chargement...</span></p>
        <p>Humidité: <span id="live-humid">Chargement...</span></p>
        <p>Qualité de l'air: <span id="live-qualite">Chargement...</span></p>
        <h1>Dernières valeurs enregistrées :</h1>
        <p>Température: <span id="stored-temp">Chargement...</span></p>
        <p>Humidité: <span id="stored-humid">Chargement...</span></p>
        <p>Qualité de l'air: <span id="stored-qualite">Chargement...</span></p>
        <button onclick="sendCommand()">Envoyer Commande</button>
    </body>
    </html>
    """

@app.route('/live_data')
def get_live_data():
    return jsonify({"temp": latest_temp, "humid": latest_humid, "qualite": latest_qualite})

@app.route('/data')
def get_data():
    conn = sqlite3.connect('ros_data.db')
    cursor = conn.cursor()
    cursor.execute("SELECT temp, humid, qualite FROM messages ORDER BY id DESC LIMIT 1")
    record = cursor.fetchone()
    conn.close()
    
    data = {"temp": "N/A", "humid": "N/A", "qualite": "N/A"}
    if record:
        data = {"temp": record[0], "humid": record[1], "qualite": record[2]}
    
    return jsonify(data)

@app.route('/send', methods=['POST'])
def send_command():
    global publisher_node
    command_text = "Commande ROS envoyée !"
    publisher_node.send_message(command_text)
    return jsonify({"message": command_text})

def start_ros2():
    executor = MultiThreadedExecutor()
    executor.add_node(subscriber_node)
    executor.add_node(publisher_node)
    executor.spin()

# Démarrage du thread ROS 2
ros_thread = threading.Thread(target=start_ros2, daemon=True)
ros_thread.start()

# Démarrage du serveur Flask
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)

