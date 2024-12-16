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


class GripperController():
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("gripper_controller")
        self.gripper_group = moveit_commander.MoveGroupCommander("panda_hand") # type: ignore
        rospy.Subscriber("/panda_gripper/cmd", data_class=std_msgs.msg.String, callback=self._receive_cmd, queue_size=10)
        self._cmd: Union[str, None] = None
    
    def _receive_cmd(self, cmd: std_msgs.msg.String):
        rospy.loginfo(f"Received new command: {cmd}")
        self._cmd = cmd.data

    def open_gripper(self):
        self.gripper_group.set_named_target('open')
        self.gripper_group.go(wait=True)

    def close_gripper(self):
        rospy.loginfo("closing!")
        self.gripper_group.set_named_target('close')
        self.gripper_group.go(wait=True)
    
    def run(self):
        while not rospy.is_shutdown():
            if self._cmd:
                if self._cmd == "open":
                    self.open_gripper()
                    rospy.sleep(2)
                if self._cmd == "close":
                    self.close_gripper()
                    rospy.sleep(2)

if __name__ == "__main__":
    gripper_controller = GripperController()
    gripper_controller.run()



