# odom_reader.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import threading

class OdomReader:
    def __init__(self):
        self.position = None
        self.lock = threading.Lock()

    def get_position(self):
        rclpy.init()
        node = rclpy.create_node('odom_reader_node')

        done_event = threading.Event()

        def odom_callback(msg):
            with self.lock:
                position = msg.pose.pose.position
                self.position = {
                    'x': position.x,
                    'y': position.y,
                    'z': position.z
                }
            done_event.set()

        subscription = node.create_subscription(
            Odometry,
            '/odom',
            odom_callback,
            10
        )

        while not done_event.is_set():
            rclpy.spin_once(node, timeout_sec=1.0)

        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

        return self.position

def get_current_position():
    odom_reader = OdomReader()
    return odom_reader.get_position()

# This function can now be imported and called from another Python file.
