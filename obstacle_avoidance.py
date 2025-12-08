#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class ObstacleAvoidance:
    def __init__(self):
        self.obstacle = False

        self.sub_obstacle = rospy.Subscriber(
            "/obstacle", Bool, self.obstacle_callback
        )
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.rate = rospy.Rate(30)  # 30 Hz

        # Right turn angular velocity (negative value indicates right turn)
        self.turn_speed = rospy.get_param("~turn_speed", -0.6)

    def obstacle_callback(self, msg: Bool):
        self.obstacle = msg.data

    def spin(self):
        twist = Twist()
        while not rospy.is_shutdown():
            if self.obstacle:
                # When obstacle detected ahead: stop forward movement, only turn right
                twist.linear.x = -0.03
                twist.angular.z = self.turn_speed
                self.pub_cmd.publish(twist)
            # When no obstacle: don't publish, let teleop's /cmd_vel take over
            self.rate.sleep()


def main():
    rospy.init_node("obstacle_avoidance")
    node = ObstacleAvoidance()
    rospy.loginfo("obstacle_avoidance node started")
    node.spin()


if __name__ == "__main__":
    main()
