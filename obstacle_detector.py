#!/usr/bin/env python3
import math

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleDetector:
    def __init__(self):
        # Publish whether there is an obstacle ahead
        self.pub_obstacle = rospy.Publisher("/obstacle", Bool, queue_size=1)

        # Subscribe to laser scan data
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Parameters: distance threshold (meters) and front angle range (degrees)
        self.distance_threshold = rospy.get_param("~distance_threshold", 0.36)
        self.front_angle_deg = rospy.get_param("~front_angle_deg", 36.0)

    def scan_callback(self, msg: LaserScan):
        # Calculate index range corresponding to front angles (assuming 0 rad is straight ahead)
        # For TurtleBot3: angle_min is typically -pi, angle_max is +pi
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        n = len(msg.ranges)

        # Index corresponding to 0 rad (straight ahead)
        center_idx = int(round((0.0 - angle_min) / angle_inc))

        # Front angles +/- front_angle_deg in radians
        half_angle_rad = math.radians(self.front_angle_deg)
        half_count = int(round(half_angle_rad / angle_inc))

        start = max(0, center_idx - half_count)
        end = min(n - 1, center_idx + half_count)

        in_front = []
        for i in range(start, end + 1):
            r = msg.ranges[i]
            # Filter out inf and NaN values
            if math.isfinite(r):
                in_front.append(r)

        obstacle = False
        if in_front:
            min_dist = min(in_front)
            obstacle = min_dist < self.distance_threshold

        self.pub_obstacle.publish(Bool(data=obstacle))


def main():
    rospy.init_node("obstacle_detector")
    node = ObstacleDetector()
    rospy.loginfo("obstacle_detector node started")
    rospy.spin()


if __name__ == "__main__":
    main()
