#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np

class ArucoDetector:
    def __init__(self):
        rospy.init_node("aruco_detector", anonymous=True)

        # Subscribers/publishers
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.pose_pub = rospy.Publisher("/aruco/pose", PoseStamped, queue_size=10)

        self.bridge = CvBridge()

        # ArUco marker definition
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # Placeholder (should calibrate later)
        self.camera_matrix = np.array([[600,   0, 320],
                                       [  0, 600, 240],
                                       [  0,   0,   1]], dtype=float)

        self.dist_coeffs = np.zeros((5, 1))  # assume no lens distortion

        rospy.loginfo("ArUco detector started.")

    def image_callback(self, msg):
        # Convert ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.parameters
        )

        if ids is not None:
            # Estimate marker pose (marker size assumed 5 cm)
            marker_size = 0.05
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                marker_size,
                self.camera_matrix,
                self.dist_coeffs
            )

            pose = PoseStamped()
            pose.header.frame_id = "camera_frame"
            pose.header.stamp = rospy.Time.now()

            # Publish first detected marker
            pose.pose.position.x = float(tvecs[0][0][0])
            pose.pose.position.y = float(tvecs[0][0][1])
            pose.pose.position.z = float(tvecs[0][0][2])

            self.pose_pub.publish(pose)

            # Draw markers for debugging
            aruco.drawDetectedMarkers(frame, corners, ids)
            aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvecs[0], tvecs[0], 0.05)

        # Debugging display (optional)
        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    ArucoDetector()
