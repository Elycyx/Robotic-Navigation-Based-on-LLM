# trajectory_length_calculator.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import threading
import math

class TrajectoryLengthCalculator(Node):
    def __init__(self):
        super().__init__('trajectory_length_calculator')
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.odom_subscription  # prevent unused variable warning
        self.distance = 0.0
        self.last_position = None

    def odom_callback(self, msg):
        # Extract position data from msg
        position = msg.pose.pose.position

        # If this is the first time, just save the position
        if self.last_position is None:
            self.last_position = position
            return

        # Calculate the distance between the last position and the current position
        delta_x = position.x - self.last_position.x
        delta_y = position.y - self.last_position.y
        segment_length = math.sqrt(delta_x**2 + delta_y**2)

        # Add the segment length to the total distance
        self.distance += segment_length

        # Update the last position
        self.last_position = position

    def get_total_distance(self):
        return self.distance

    def reset_distance(self):
        self.distance = 0.0
        self.last_position = None

# Global reference to the calculator node
calculator_node = None
# Thread for spinning the node
spin_thread = None

def start_trajectory_length_calculator():
    global calculator_node, spin_thread
    # rclpy.init()
    calculator_node = TrajectoryLengthCalculator()

    def spin_node():
        rclpy.spin(calculator_node)

    spin_thread = threading.Thread(target=spin_node)
    spin_thread.start()

def stop_trajectory_length_calculator():
    global calculator_node, spin_thread
    if calculator_node is not None:
        total_distance = calculator_node.get_total_distance()
        calculator_node.reset_distance()
        calculator_node.destroy_node()
        calculator_node = None
        rclpy.shutdown()
        if spin_thread.is_alive():
            spin_thread.join()  # Ensure the spin thread has finished
        spin_thread = None
        return total_distance
    else:
        return None

if __name__ == '__main__':
    # This allows the file to be executed directly, which will start the calculator.
    start_trajectory_length_calculator()
