#!/usr/bin/env python3
import sys

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("pick_place_controller")
    rospy.loginfo("Pick Place Controller Started ...")

    # robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)


