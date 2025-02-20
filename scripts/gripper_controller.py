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

from franka_gripper.msg import MoveActionGoal, MoveActionResult, GraspActionGoal, GraspActionResult
from franka_msgs.msg import FrankaState

# See this: https://frankaemika.github.io/docs/franka_ros.html?highlight=graspactiongoal#pick-place-example

class GripperController():
    def __init__(self) -> None:
        self.grip_move_publisher =          rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=10)
        self.grip_moveresult_callback =     rospy.Publisher('/franka_gripper/move/result', MoveActionResult, self.moveresult_callback)
        self.grip_moveresult = MoveActionResult() # see: rosmsg show franka_gripper/MoveActionResult

        self.grip_grasp_publisher =         rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=10)
        self.grip_graspresult_callback =    rospy.Publisher('/franka_gripper/grasp/result', GraspActionResult, self.graspresult_callback)
        self.grip_graspresult = GraspActionResult() # see: rosmsg show franka_gripper/GraspActionResult
        rospy.sleep(3)

    def moveresult_callback(self, result):
        rospy.loginfo("MoveActionResult received!", result)
        self.grip_moveresult = result

    def graspresult_callback(self, result):
        rospy.loginfo("GraspActionResult received!", result)
        self.grip_graspresult = result


    def set_width(self, width, speed=0.15):
        
        mag = MoveActionGoal()

        mag.goal.width = width # 0.00 < width < 0.08

        mag.goal.speed = speed
        
        rospy.loginfo(f"Opening Gripper to width {width}...")

        self.grip_move_publisher.publish(mag)
        rospy.sleep(4) # Wait for Gripper to register message and open 
        print(self.grip_moveresult.result.success)
        return self.grip_moveresult.result.success # .success is a bool

        

    def grasp(self):

        gag = GraspActionGoal()

        gag.goal.width = 0.040 # cube size

        # Grasp is successful if distance between fingers lies between .epsilon.inner and .epsilon.outer + .width
        gag.goal.epsilon.inner = 0.01
        gag.goal.epsilon.outer = 0.01

        gag.goal.speed = 0.15
        gag.goal.force = 30.0 # N

        rospy.loginfo(f"Grasping with {gag.goal.force} N...")
        
        self.grip_grasp_publisher.publish(gag)
        rospy.sleep(4)
        print(self.grip_graspresult.result.success)

        return self.grip_graspresult.result.success # .success is a bool

        