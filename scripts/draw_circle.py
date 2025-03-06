#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def draw_circle(msg: Pose):
    pos = Twist()
    pos.linear.x = 1.0
    pos.angular.z = 5.0

    pub.publish(pos)



if __name__ == "__main__":
    rospy.init_node("draw_circle")
    rospy.loginfo("Draw Circle Node Started ...")

    sub = rospy.Subscriber("/turtle1/pose", data_class=Pose, callback=draw_circle, queue_size=10)

    pub = rospy.Publisher("/turtle1/cmd_vel", data_class=Twist, queue_size=10)

    rate = rospy.Rate(2)


    rospy.spin()


