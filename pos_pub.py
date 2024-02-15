from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import rclpy

class Nav2Client(Node):

    def __init__(self):
        super().__init__('nav2_client')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal_and_wait(self, x, y, z, w):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.w = w
        goal_msg.pose.header.frame_id = 'map'

        self._action_client.wait_for_server()
        self.future = self._action_client.send_goal_async(goal_msg)

        # Wait for the server to accept the goal
        rclpy.spin_until_future_complete(self, self.future)
        goal_handle = self.future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Wait for the goal to be done
        self.result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)

        result = self.result_future.result().result
        self.get_logger().info('Goal completed')
        return result
    def nav2(self, position):
        result = self.send_goal_and_wait(position[0], position[1], 0.0, 1.0)
        return result



def main(args=None):
    rclpy.init(args=args)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
