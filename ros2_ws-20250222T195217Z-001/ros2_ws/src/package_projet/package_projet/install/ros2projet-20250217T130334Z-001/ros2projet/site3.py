import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from flask import Flask, jsonify, request
import threading
import sqlite3

# Variable pour stocker le dernier message reçu
latest_message = "Aucun message reçu"

# ROS 2 Subscriber Node
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('site_flask_int')
        self.subscription = self.create_subscription(
            String,
            'topic',  # Nom du topic à écouter
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        global latest_message
        latest_message = msg.data
        self.get_logger().info(f"Message reçu: {msg.data}")

        # Enregistrement dans la base de données
        conn = sqlite3.connect('ros_data.db')
        cursor = conn.cursor()
        cursor.execute("INSERT INTO messages (data) VALUES (?)", (msg.data,))
        conn.commit()
        conn.close()

# ROS 2 Publisher Node
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('ste_flask_out')
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
            function fetchData() {
                fetch('/data')
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById('ros-message').innerText = data.message;
                    });
            }
            function sendMessage() {
                fetch('/send', { method: 'POST' })
                    .then(response => response.json())
            }
            setInterval(fetchData, 1000);
        </script>
    </head>
    <body>
        <h1>Derniers messages ROS 2 :</h1>
        <ul id="ros-messages"></ul>
        <button onclick="sendMessage()">Envoyer Commande</button>
        <script>
            function fetchMessages() {
                fetch('/data')
                    .then(response => response.json())
                    .then(data => {
                        let messagesList = document.getElementById('ros-messages');
                        messagesList.innerHTML = '';
                        data.messages.forEach(msg => {
                            let li = document.createElement('li');
                            li.innerText = msg;
                            messagesList.appendChild(li);
                        });
                    });
            }
            setInterval(fetchMessages, 1000);
        </script>
    </body>
    </html>
    """

@app.route('/data')
def get_data():
    conn = sqlite3.connect('ros_data.db')
    cursor = conn.cursor()
    cursor.execute("SELECT data FROM messages ORDER BY id DESC LIMIT 50")
    messages = [row[0] for row in cursor.fetchall()]
    conn.close()
    return jsonify({"messages": messages})

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

