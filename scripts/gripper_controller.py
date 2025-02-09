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

from franka_gripper.msg import MoveActionGoal, GraspActionGoal


class GripperController():
    def __init__(self) -> None:
        self.grip_move_publisher = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=10)
        self.grip_grasp_publisher = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=10)
    
    def set_width(self, width):
        
        mag = MoveActionGoal()
        
        mag.goal.width = width # 0.00 < width < 0.08

        mag.goal.speed = 0.15
        
        rospy.loginfo(f"Opening Gripper to width {width}...")

        self.grip_move_publisher.publish(mag)

        

    def grasp(self):

        gag = GraspActionGoal()

        gag.goal.width = 0.045 # cube size

        # Grasp is successful if distance between fingers lies between .epsilon.inner and .epsilon.outer + .width
        gag.goal.epsilon.inner = 0.1
        gag.goal.epsilon.outer = 0.1

        gag.goal.speed = 0.15
        gag.goal.force = 10.0 # N

        rospy.loginfo(f"Grasping with {gag.goal.force} N...")
        
        self.grip_grasp_publisher.publish(gag)


        