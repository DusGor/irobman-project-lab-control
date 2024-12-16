#!/usr/bin/env python3
import sys
from typing import Union
import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

from nav_msgs.msg import Odometry


class PickPlaceController:
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_place_controller")
        rospy.loginfo("Pick Place Controller Started ...")

        self.robot = moveit_commander.RobotCommander() # type: ignore
        self.scene = moveit_commander.PlanningSceneInterface() # type: ignore
        group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name) # type: ignore

        rospy.Subscriber("/cube_0_odom", data_class=Odometry, callback=self._get_cube_info, queue_size=10)

        self.gripper_cmd_pub = rospy.Publisher("/panda_gripper/cmd", data_class=std_msgs.msg.String, queue_size=10)


        self.cube_pose: Union[geometry_msgs.msg.Pose, None] = None

    def _get_cube_info(self, odo: Odometry):
        self.cube_pose = odo.pose.pose

    def _close_gripper(self):
        self.gripper_cmd_pub.publish("close")
        rospy.sleep(2)
    
    def _open_gripper(self):
        self.gripper_cmd_pub.publish("open")
        rospy.sleep(2)

    # def _set_gripper_pos(self, value):
    #     # TODO: Does not seem to be the most elegant way. We should rework this.
    #     # Source: http://docs.ros.org/en/jade/api/moveit_commander/html/robot_8py_source.html
    #     if not (0.0 <= value <= 1.0):
    #         raise ValueError(f"Gripper Pos must be between 0.0 and 1.0, was {value}!") 
    #     joint1: moveit_commander.RobotCommander.Joint = self.robot.get_joint("panda_finger_joint1")
    #     j1_min = joint1.min_bound()
    #     j1_max = joint1.max_bound()
    #     joint1.move((j1_max - j1_min) * value, wait=True)

    #     joint2: moveit_commander.RobotCommander.Joint = self.robot.get_joint("panda_finger_joint2")
    #     j2_min = joint2.min_bound()
    #     j2_max = joint2.max_bound()
    #     joint2.move((j2_max - j2_min) * value, wait=True)

    def move_to_pose(self, pose: geometry_msgs.msg.Pose):
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def pick(self):
        if self.cube_pose is None:
            rospy.logwarn("Cube position is None!")
            return
        
        # Move to pre-grasp pose
        target_pose: geometry_msgs.msg.Pose = self.cube_pose
        target_pose.orientation.x = 1.0
        target_pose.orientation.y = 0
        target_pose.orientation.z = 0
        target_pose.orientation.w = 0
        target_pose.position.z += 0.2 # offset
        self.move_to_pose(target_pose)
        rospy.sleep(2)

        # Open gripper
        self._open_gripper()

        # Move to grasp pose
        target_pose: geometry_msgs.msg.Pose = self.cube_pose
        target_pose.orientation.x = 1.0
        target_pose.orientation.y = 0
        target_pose.orientation.z = 0
        target_pose.orientation.w = 0
        target_pose.position.z += 0.1 # offset
        self.move_to_pose(target_pose)

        

        # Close gripper
        self._close_gripper()

        # Lift object
        target_pose: geometry_msgs.msg.Pose = self.cube_pose
        target_pose.orientation.x = 1.0
        target_pose.orientation.y = 0
        target_pose.orientation.z = 0
        target_pose.orientation.w = 0
        target_pose.position.z += 0.3 # offset
        self.move_to_pose(target_pose)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Wait for cube_pose to be available
            if self.cube_pose:
                self.pick()
                rospy.sleep(2)
                break



if __name__ == "__main__":
    start_pose = geometry_msgs.msg.Pose()
    # start_pose.orientation.x = np.sin(np.pi/2)
    # start_pose.orientation.y = 0
    # start_pose.orientation.z = 0
    # start_pose.orientation.w = np.cos(np.pi/2)
    start_pose.orientation.x = 1.0
    start_pose.orientation.y = 0
    start_pose.orientation.z = 0
    start_pose.orientation.w = 0

    start_pose.position.x = 0.3
    start_pose.position.y = 0.0
    start_pose.position.z = 0.5

    
    # cube_pose: geometry_msgs.msg.Pose = odo.pose.pose
    # position: geometry_msgs.msg.Point = cube_pose.position
    # orientation: geometry_msgs.msg. Quaternion = cube_pose.orientation

    # z_offset = 0.2
    # position.z += z_offset

    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.0
    # pose_goal.position = position

    # move_group.set_pose_target(pose_goal)

    # success = move_group.go(wait=True)

    # move_group.stop()

    # move_group.clear_pose_targets()

    # move_group.set_pose_target(start_pose)

    # success = move_group.go(wait=True)

    # move_group.stop()

    # move_group.clear_pose_targets()
    controller = PickPlaceController()
    controller.move_to_pose(start_pose)
    controller.run()
    # rospy.spin()

    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = 0.4
    # pose_goal.position.y = 0.1
    # pose_goal.position.z = 0.4

    # move_group.set_pose_target(pose_goal)

    # success = move_group.go(wait=True)

    # move_group.stop() # Ensures there is no residual movement

    # move_group.clear_pose_targets() # always good to clear pose targets

    # current_pose = move_group.get_current_pose().pose

    # rospy.loginfo(current_pose)

