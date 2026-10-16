#!/usr/bin/env python3
"""
shelf_detector.py — detects shelf rail in the depth image and publishes its pose.

Subscribes:
  /camera/depth_image  (sensor_msgs/Image, 32FC1 depth in metres)
  /camera/image_raw    (sensor_msgs/Image, RGB)

Publishes:
  /shelf_detector/rail_pose   (geometry_msgs/PoseStamped)
      → pose of the nearest vertical upright rail in camera frame
  /shelf_detector/status      (std_msgs/String)
      → "SEARCHING" | "RAIL_FOUND" | "ALIGNED"

Algorithm (simulation-grade, no real ML needed):
  1. Read the central column of the depth image.
  2. Find the column with minimum average depth in a horizontal ROI at
     the expected shelf-beam height (0.3–1.5 m above ground).
  3. Estimate the lateral offset from the image centre (= yaw error).
  4. Publish rail pose in camera_optical_link frame.
  5. The climb_controller subscribes to rail_pose and drives toward it.
"""

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge


class ShelfDetector(Node):
    def __init__(self):
        super().__init__('shelf_detector')

        # Parameters
        self.declare_parameter('min_depth', 0.20)   # ignore anything closer than 20 cm
        self.declare_parameter('max_depth', 4.00)   # ignore anything farther than 4 m
        self.declare_parameter('rail_height_m', 0.80) # expected shelf beam height (world Z)
        self.declare_parameter('img_width',  640)
        self.declare_parameter('img_height', 480)

        self.bridge = CvBridge()
        self._latest_depth = None
        self._status = "SEARCHING"

        # Subscribers
        self.create_subscription(Image, '/camera/depth_image',
                                 self._depth_cb, 10)

        # Publishers
        self._rail_pub  = self.create_publisher(PoseStamped, '/shelf_detector/rail_pose', 10)
        self._status_pub = self.create_publisher(String,      '/shelf_detector/status',   10)

        # 10 Hz detection loop
        self.create_timer(0.1, self._detect)
        self.get_logger().info('ShelfDetector started')

    def _depth_cb(self, msg: Image):
        try:
            self._latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f'depth_cb error: {e}')

    def _detect(self):
        if self._latest_depth is None:
            self._publish_status("SEARCHING")
            return

        depth = self._latest_depth
        h, w = depth.shape

        min_d = self.get_parameter('min_depth').value
        max_d = self.get_parameter('max_depth').value

        # Mask invalid values
        valid = np.where((depth > min_d) & (depth < max_d), depth, np.nan)

        # ROI: middle third of image height (where shelf beams appear)
        row_lo = h // 3
        row_hi = 2 * h // 3
        roi = valid[row_lo:row_hi, :]

        # Column-wise mean depth in ROI
        col_mean = np.nanmean(roi, axis=0)
        if np.all(np.isnan(col_mean)):
            self._publish_status("SEARCHING")
            return

        # Nearest column = most likely upright / object face
        best_col = int(np.nanargmin(col_mean))
        best_depth = float(col_mean[best_col])

        if best_depth > max_d:
            self._publish_status("SEARCHING")
            return

        # Pixel offset from image centre → lateral angle
        cx = w / 2.0
        fov_h = 1.047  # radians (60 deg, must match URDF sensor)
        fx = w / (2.0 * np.tan(fov_h / 2.0))
        lateral_offset_m = (best_col - cx) / fx * best_depth

        # Build PoseStamped in camera_optical_link frame:
        #   z = forward (depth to rail)
        #   x = rightward lateral offset
        #   y = 0 (no height correction yet — add later with tf2)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'camera_optical_link'
        pose.pose.position.x = float(lateral_offset_m)
        pose.pose.position.y = 0.0
        pose.pose.position.z = float(best_depth)
        pose.pose.orientation.w = 1.0

        self._rail_pub.publish(pose)

        aligned = abs(lateral_offset_m) < 0.05 and best_depth < 0.50
        status = "ALIGNED" if aligned else "RAIL_FOUND"
        self._publish_status(status)

    def _publish_status(self, s: str):
        self._status = s
        msg = String()
        msg.data = s
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShelfDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
