#This node will use OpenCV and ROS2 to detect cones in the camera feed, classify their colors, and publish the closest detected cone.
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class ConeDetector(Node):
    def __init__(self):
        super().__init__("cone_detector")

        # Publisher for detected cone color
        self.publisher_ = self.create_publisher(String, "/detected_cone", 10)

        # Subscribe to RGB and Depth Camera Topics
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback, 10
        )

        # Store images for processing
        self.rgb_image = None
        self.depth_image = None

    def image_callback(self, msg):
        """Callback for RGB image"""
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        """Callback for Depth image"""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def detect_cone(self):
        """Detect cones using RGB and Depth Data"""
        if self.rgb_image is None or self.depth_image is None:
            return

        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)

        # Define color ranges for cone detection
        color_ranges = {
            "yellow": [(20, 100, 100), (30, 255, 255)],
            "blue": [(100, 150, 50), (140, 255, 255)],
            "red": [(0, 120, 70), (10, 255, 255)],
            "ramp": [(10, 20, 20), (20, 255, 255)],
        }

        closest_cone = None
        min_distance = float("inf")

        for color, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 500:  # Ignore small objects
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = h / w

                # Filter objects that don't look like cones
                if not (1.5 < aspect_ratio < 4):
                    continue

                # Extract depth from the depth image at the cone's center
                cone_center_x = x + w // 2
                cone_center_y = y + h // 2
                depth_value = self.depth_image[cone_center_y, cone_center_x]

                # Ignore invalid depth values
                if depth_value == 0 or np.isnan(depth_value):
                    continue

                # Prioritize the closest detected cone
                if depth_value < min_distance:
                    min_distance = depth_value
                    closest_cone = color

        if closest_cone:
            msg = String()
            msg.data = closest_cone
            self.publisher_.publish(msg)
            self.get_logger().info(f"Detected closest cone: {closest_cone} at {min_distance:.2f} meters")

    def run(self):
        while rclpy.ok():
            self.detect_cone()

def main():
    rclpy.init()
    node = ConeDetector()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
