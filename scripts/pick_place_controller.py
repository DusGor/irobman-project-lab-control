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


    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    success = move_group.go(wait=True)

    move_group.stop() # Ensures there is no residual movement

    move_group.clear_pose_targets() # always good to clear pose targets

    current_pose = move_group.get_current_pose().pose

    rospy.loginfo(current_pose)

