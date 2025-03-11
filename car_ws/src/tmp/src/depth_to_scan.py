#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, LaserScan, CameraInfo, PointCloud2
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel

class DepthToLaserScan:
    def __init__(self):
        rospy.init_node("depth_to_scan", anonymous=True)

        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Camera model (will be updated dynamically)
        self.camera_model = PinholeCameraModel()

        # Subscribers
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.camera_info_callback)

        # Publisher
        self.scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=10)

        self.camera_info_received = False  # Flag to ensure we have camera info before processing depth images

    def camera_info_callback(self, msg):
        """Callback to update camera intrinsics."""
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_received = True

    def depth_callback(self, msg):
        """Converts depth image to LaserScan."""
        if not self.camera_info_received:
            rospy.logwarn("Waiting for camera info...")
            return
        
        # Convert depth image to CV format
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

        # Get image dimensions
        height, width = depth_image.shape
        center_row = height // 2  # Taking a horizontal slice in the middle
        
        # Get horizontal row of depth values
        depth_row = depth_image[center_row, :]

        # Prepare LaserScan message
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = "camera_link"
        scan.angle_min = -self.camera_model.cx() * self.camera_model.fx() ** -1  # Leftmost angle
        scan.angle_max = (width - self.camera_model.cx()) * self.camera_model.fx() ** -1  # Rightmost angle
        scan.angle_increment = (scan.angle_max - scan.angle_min) / width
        scan.range_min = 0.1  # Min valid range (adjust as needed)
        scan.range_max = 5.0   # Max valid range (adjust as needed)
        scan.ranges = []

        # Convert depth row to LaserScan format
        for i in range(width):
            depth = depth_row[i]
            if np.isfinite(depth) and depth > scan.range_min and depth < scan.range_max:
                scan.ranges.append(depth)
            else:
                scan.ranges.append(float('inf'))  # Mark invalid values as max range
        
        # Publish LaserScan message
        self.scan_pub.publish(scan)

if __name__ == "__main__":
    try:
        DepthToLaserScan()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
