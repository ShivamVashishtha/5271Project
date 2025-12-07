#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()

        # ROS 2 subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # ROS 2 publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)

        # Detector parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # Initial camera matrix (placeholder)
        self.camera_matrix = np.array([[600, 0, 320],
                                       [0, 600, 240],
                                       [0,   0,   1]], dtype=float)

        self.dist_coeffs = np.zeros((5, 1))

        self.get_logger().info("🔍 ArUco detector started.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, self.parameters)

        if ids is not None:
            marker_size = 0.05  # meters
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                marker_size,
                self.camera_matrix,
                self.dist_coeffs
            )

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'camera_frame'

            # publish translation of first marker
            pose_msg.pose.position.x = float(tvecs[0][0][0])
            pose_msg.pose.position.y = float(tvecs[0][0][1])
            pose_msg.pose.position.z = float(tvecs[0][0][2])

            self.pose_pub.publish(pose_msg)

            # Debug visualization only
            aruco.drawDetectedMarkers(frame, corners, ids)
            aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvecs[0], tvecs[0], marker_size)

        cv2.imshow("ArUco Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
