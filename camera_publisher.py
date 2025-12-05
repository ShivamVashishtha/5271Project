#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node("camera_publisher", anonymous=True)

    # Publisher for camera image
    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)

    bridge = CvBridge()

    # Attempt to open Pi camera video stream
    # For USB cam change 0 → 1
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Could not open Pi camera.")
        return

    rospy.loginfo("Camera publisher started.")

    rate = rospy.Rate(15)  # 15 FPS

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to grab frame.")
            continue

        # Convert OpenCV image → ROS Image msg
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        pub.publish(msg)
        rate.sleep()

    cap.release()

if __name__ == "__main__":
    main()
