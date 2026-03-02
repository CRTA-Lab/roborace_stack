import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry  # Uvozimo Odometry poruku
from os.path import expanduser, join, exists
import os
import time
# import math # Nije više potrebno ako gledamo samo x brzinu

# --- Postavke putanje datoteke ---
home = expanduser('~')
folder = join(home, 'f1tenth_mrc_ws')
os.makedirs(folder, exist_ok=True) # Kreira folder ako ne postoji

# Novi sustav imenovanja datoteka
base_name = 'raceline'
filename = base_name + '.csv'
filepath = join(folder, filename)

version = 1
while exists(filepath): # Provjerava postoji li datoteka i dodaje verziju
    filename = f"{base_name}_{version}.csv" # npr. raceline_1.csv, raceline_2.csv
    filepath = join(folder, filename)
    version += 1

file = open(filepath, 'w') # Otvara datoteku za pisanje

class Logger(Node):
    def __init__(self):
        super().__init__("optitrack_waypoint_logger")
        
        # Pretplata na /optitrack/f1tenth/pose temu tipa PoseStamped
        self.pose_sub = self.create_subscription(PoseStamped, "/optitrack/f1tenth/pose", self.waypoint_saver, 10)
        
        # Nova pretplata na /odom temu tipa Odometry
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        
        self.last_saved_time = 0.0  # Vrijeme zadnjeg spremanja
        self.save_interval = 0.15 # Fiksni interval spremanja
        self.current_speed_x = 0.0 # Nova: Pohranjuje trenutnu brzinu po X osi
        self.speed_threshold = 0.1 # Novi: Minimalna brzina za početak snimanja (npr. 0.1 m/s)

        self.get_logger().info("Pokrećem 'Optitrack Waypoint Logger'...")

    # Nova: Callback za Odometry poruke za dobivanje brzine robota
    def odom_callback(self, msg):
        # Dobivamo linearnu brzinu po X osi
        self.current_speed_x = msg.twist.twist.linear.x

    def waypoint_saver(self, msg):
        current_time = time.time()
        
        # Provjeravamo je li prošlo dovoljno vremena od zadnjeg spremanja I je li robot u pokretu
        # Koristimo apsolutnu vrijednost brzine po X osi jer robot može ići naprijed ili natrag
        if current_time - self.last_saved_time < self.save_interval or \
           abs(self.current_speed_x) < self.speed_threshold: # Nova: Provjera praga brzine
            return  # Preskačemo spremanje

        # Dohvaćanje pozicije iz PoseStamped poruke
        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        default_value = 1.2 # Fiksna vrijednost za spremanje

        # Zapisivanje x, y i fiksne vrijednosti u CSV datoteku
        file.write('%f, %f, %f\n' % (
            pos_x,
            pos_y,
            default_value
        ))
        self.last_saved_time = current_time # Ažurira vrijeme zadnjeg spremanja

def main(args=None):
    rclpy.init(args=args) # Inicijalizira ROS2
    node = Logger() # Stvara instancu Logger node-a

    try:
        rclpy.spin(node) # Pokreće node dok se ne prekine
    except KeyboardInterrupt:
        print("Napuštam 'Optitrack Waypoint Logger'...")
    finally:
        file.close() # Zatvara datoteku
        node.destroy_node() # Uništava node
        rclpy.shutdown() # Gasi ROS2
        print(f"Datoteka spremljena na: {filepath}") # Poruka o spremanju datoteke

if __name__ == "__main__":
    main()