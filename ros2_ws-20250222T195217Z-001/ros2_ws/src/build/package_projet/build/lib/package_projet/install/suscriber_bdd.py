import rclpy
from rclpy.node import Node
import sqlite3
from std_msgs.msg import String  # Remplace par le bon type de message

class TopicToDatabase(Node):
    def __init__(self):
        super().__init__('topic_to_database')
        self.subscription = self.create_subscription(
            String,  # Remplace par le bon type de message
            'mon_topic',  # Nom du topic
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Connexion à la base de données SQLite
        self.conn = sqlite3.connect('ros_data.db')
        self.cursor = self.conn.cursor()
        self.create_table()

    def create_table(self):
        # Création de la table si elle n'existe pas
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS messages (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                data TEXT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        self.conn.commit()

    def listener_callback(self, msg):
        self.get_logger().info(f'Reçu: {msg.data}')
        self.cursor.execute("INSERT INTO messages (data) VALUES (?)", (msg.data,))
        self.conn.commit()

    def destroy_node(self):
        self.conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TopicToDatabase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

