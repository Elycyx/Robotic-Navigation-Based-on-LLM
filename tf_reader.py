import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TransformNode(Node):
    def __init__(self):
        super().__init__('transform_node3')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.x = None
        self.y = None
        self.quit = False
        self.create_timer(1, self.transform_callback)

    def transform_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
            # self.get_logger().info(f"Transform: {trans}")
            self.x = trans.transform.translation.x
            self.y = trans.transform.translation.y
            self.quit = True  # Set the flag to True to exit the spin loop
        except TransformException as ex:
            a = 1
            # self.get_logger().info(f'Could not transform map to base_footprint: {ex}')

    def get_robot_map_position(self):
        return (self.x, self.y)

    def run(self):
        while rclpy.ok() and not self.quit:
            rclpy.spin_once(self, timeout_sec=0.5)

def get_current_position(args=None):
    rclpy.init(args=args)
    transform_node = TransformNode()
    transform_node.run()
    x = transform_node.x
    y = transform_node.y

    # Correctly handle the destruction of the node and shutdown of rclpy
    transform_node.destroy_node()
    rclpy.shutdown()
    return (x, y)

