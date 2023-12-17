import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_transform(self, target_frame, source_frame):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(target_frame
, source_frame, rclpy.time.Time())
            return transform_stamped
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"LookupException: {e}")
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"ExtrapolationException: {e}")
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f"ConnectivityException: {e}")
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"TransformException: {e}")
        
        return None

def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    try:
        while rclpy.ok():
            transform_stamped = node.get_transform(target_frame='base_link', source_frame='map')
            if transform_stamped:
                # Access the translation and rotation components of the transform
                translation = transform_stamped.transform.translation
                rotation = transform_stamped.transform.rotation

                node.get_logger().info(f"Translation: {translation.x}, {translation.y}, {translation.z}")
                node.get_logger().info(f"Rotation: {rotation.x}, {rotation.y}, {rotation.z}, {rotation.w}")

            rclpy.spin_once(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
