import rclpy
import random
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class RandomPosePublisher(Node):
    def __init__(self):
        super().__init__('random_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/target_pose', 10)

    def publish_random_pose(self):
        while rclpy.ok():
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.pose.position.x = random.uniform(0.2, 0.8)
            msg.pose.position.y = random.uniform(-0.5, 0.5)
            msg.pose.position.z = random.uniform(0.1, 0.5)
            msg.pose.orientation.w = 1.0  # Keep orientation fixed

            self.publisher.publish(msg)
            self.get_logger().info(f'Published Random Pose: {msg.pose.position}')
            time.sleep(1)  # Adjust publishing rate

def main():
    rclpy.init()
    node = RandomPosePublisher()
    node.publish_random_pose()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
