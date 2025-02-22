import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import sqlite3
from datetime import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.create_subscription(Int32, 'temp', self.temp_callback, 10)
        self.create_subscription(Int32, 'humid', self.humid_callback, 10)
        self.create_subscription(Int32, 'qualite', self.qualite_callback, 10)
        self.create_subscription(String, 'bdd', self.bdd_callback, 10)

        self.latest_temp = None
        self.latest_humid = None
        self.latest_qualite = None
        self.heure= None
        self.data_buffer = []
        self.db_init()

    def db_init(self):
        conn = sqlite3.connect('ros_data.db')
        cursor = conn.cursor()
        cursor.execute('''CREATE TABLE IF NOT EXISTS messages (
                            id INTEGER PRIMARY KEY AUTOINCREMENT,
                            temp INTEGER,
                            humid INTEGER,
                            qualite INTEGER,
                            heure TEXT
                            )''')
        conn.commit()
        conn.close()

    def save_to_db(self):
        if len(self.data_buffer) >= 100000:
            conn = sqlite3.connect('ros_data.db')
            cursor = conn.cursor()
            cursor.executemany("INSERT INTO messages (temp, humid, qualite, heure) VALUES (?, ?, ?, ?)", self.data_buffer)
            conn.commit()
            conn.close()
            self.data_buffer = []
            self.get_logger().info("100000 valeurs enregistrées dans la base de données")

    def temp_callback(self, msg):
        self.latest_temp = msg.data
        self.check_and_store()
        self.get_logger().info(f"Température reçue: {msg.data}")

    def humid_callback(self, msg):
        self.latest_humid = msg.data
        self.check_and_store()
        self.get_logger().info(f"Humidité reçue: {msg.data}")

    def qualite_callback(self, msg):
        self.latest_qualite = msg.data
        self.check_and_store()
        self.get_logger().info(f"Qualité de l'air reçue: {msg.data}")
        
    def bdd_callback(self, msg):
        conn = sqlite3.connect('ros_data.db')
        cursor = conn.cursor()
        cursor.executemany("INSERT INTO messages (temp, humid, qualite, heure) VALUES (?, ?, ?, ?)", self.data_buffer)
        conn.commit()
        conn.close()
        self.data_buffer = []
        self.get_logger().info("Valeurs enregistrées dans la base de données")

    def check_and_store(self):
        if self.latest_temp is not None and self.latest_humid is not None and self.latest_qualite is not None:
            heure = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            self.data_buffer.append((self.latest_temp, self.latest_humid, self.latest_qualite, heure))
            self.get_logger().info(f"Stockage en mémoire tampon : {self.latest_temp}, {self.latest_humid}, {self.latest_qualite}, {self.heure}")
            
            # Réinitialiser pour éviter les doublons jusqu'à la prochaine série de valeurs
            self.latest_temp = None
            self.latest_humid = None
            self.latest_qualite = None
            
            self.save_to_db()

# Initialisation du nœud ROS2
rclpy.init()
node = DataLogger()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
