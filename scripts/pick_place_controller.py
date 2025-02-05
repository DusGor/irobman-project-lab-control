#!/usr/bin/env python3
import sys
from typing import List, Union
import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import shape_msgs.msg
import trajectory_msgs.msg

from nav_msgs.msg import Odometry


class PickPlaceController:
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_place_controller")
        rospy.loginfo("Pick Place Controller Started ...")

        self.robot = moveit_commander.RobotCommander() # type: ignore
        self.scene = moveit_commander.PlanningSceneInterface() # type: ignore
        group_name = "panda_arm"
        self.move_group: moveit_commander.MoveGroupCommander.MoveGroupCommander = moveit_commander.MoveGroupCommander(group_name) # type: ignore
        # self.move_group.set_planning_time(45)
        self.tau = 2 * np.pi # type: ignore

        self._planning_frame = self.robot.get_planning_frame()

        print(f"Planning frame: {self._planning_frame}")

        rospy.Subscriber("/cube_0_odom", data_class=Odometry, callback=self._get_cube_info, queue_size=10)

        self.gripper_cmd_pub = rospy.Publisher("/panda_gripper/cmd", data_class=std_msgs.msg.String, queue_size=10)

        self.cube_pose: Union[geometry_msgs.msg.Pose, None] = None

        rospy.sleep(2)

        self._add_collision_objects()

    def _get_cube_poses(self) -> List[Odometry]:
        return [self.cube_pose] # type: ignore

    def _get_cube_info(self, odo: Odometry):
        self.cube_pose = odo.pose.pose

    def _close_gripper(self):
        self.gripper_cmd_pub.publish("close")
        rospy.sleep(2)
    
    def _open_gripper(self, pre_grasp_posture: trajectory_msgs.msg.JointTrajectory):
        pre_grasp_posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        pre_grasp_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
        pre_grasp_posture.points[0].positions = [0.04, 0.04]
        pre_grasp_posture.points[0].time_from_start = rospy.Duration(nsecs=int(5e8)) # type: ignore
    
    def _closed_gripper(self, grasp_posture: trajectory_msgs.msg.JointTrajectory):
        grasp_posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        grasp_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
        grasp_posture.points[0].positions = [0.00, 0.00]
        grasp_posture.points[0].time_from_start = rospy.Duration(nsecs=int(5e8)) # type: ignore
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

    def _create_collision_object(self, id, dimensions, pose: Union[geometry_msgs.msg.Pose, Odometry],):
        obj = moveit_msgs.msg.CollisionObject()
        obj.id = id
        obj.header.frame_id = self._planning_frame

        solid = shape_msgs.msg.SolidPrimitive()
        solid.type = solid.BOX
        solid.dimensions = dimensions
        obj.primitives = [solid]

        obj_pose = geometry_msgs.msg.Pose()
        obj_pose.position.x = pose.position.x
        obj_pose.position.y = pose.position.y
        obj_pose.position.z = pose.position.z
        
        obj_pose.orientation.w = pose.orientation.w
        obj_pose.orientation.x = pose.orientation.x
        obj_pose.orientation.y = pose.orientation.y
        obj_pose.orientation.z = pose.orientation.z

        obj.primitive_poses = [obj_pose]
        obj.operation = obj.ADD
        return obj

    def _add_collision_objects(self):
        # SCENE = moveit_commander.PlanningSceneInterface()

        cube_size  = 0.045

        table_size = (0.81, 1.49, 0.787)


        table_pose = geometry_msgs.msg.Pose()
        table_pose.position.x = 0.495
        table_pose.position.y = 0.0
        table_pose.position.z = -0.3935

        table = self._create_collision_object(id='table',
                                        dimensions=table_size,
                                        pose=table_pose)
        self.scene.add_object(table)

        for i, cube_pose in enumerate(self._get_cube_poses()):
            if cube_pose is not None:
                msg = f"Creating collision object for cube {i} at pos {cube_pose}"
                rospy.loginfo(msg)
                cube = self._create_collision_object(id=f'cube_{i}', dimensions=[cube_size, cube_size, cube_size], pose=cube_pose)
                self.scene.add_object(cube)

        

    def move_to_pose(self, pose: geometry_msgs.msg.Pose):
        rospy.loginfo(f"Target Pose: {pose}")
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()


    def _pick(self):

        if (cube_pose := self._get_cube_poses()[0]) is not None:
            # self.move_to_pose(cube_pose)
            rospy.loginfo(f"Planning Grasp for cube ...")
            # * Set grasp
            grasp = moveit_msgs.msg.Grasp()
            grasp.grasp_pose.header.frame_id = self._planning_frame
            grasp_pose: geometry_msgs.msg.Pose = grasp.grasp_pose.pose
            # TODO: Set Orientation -> grasp_pose.orientation
            grasp_pose.position = cube_pose.position

            # * Setting pre-grasp approach
            grasp.pre_grasp_approach.direction.header.frame_id = self._planning_frame
            # Direction is set as positive z axis
            grasp.pre_grasp_approach.direction.vector.z = 1.0
            grasp.pre_grasp_approach.min_distance = 0.095
            grasp.pre_grasp_approach.desired_distance = 0.115

            # * Set post-grasp retreat
            grasp.post_grasp_retreat.direction.header.frame_id = self._planning_frame
            # Direction is set as positive z axis
            grasp.post_grasp_retreat.direction.vector.z = 1.0
            grasp.post_grasp_retreat.min_distance = 0.1
            grasp.post_grasp_retreat.desired_distance = 0.25

            # * Setting posture of ee before grasp
            self._open_gripper(grasp.pre_grasp_posture)

            # * Set posture of ee during grasp
            self._closed_gripper(grasp.grasp_posture)

            self.move_group.set_support_surface_name("table")

            self.move_group.pick("cube_0", grasp)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Wait for cube_pose to be available
            if self.cube_pose:
                self._pick()
                rospy.sleep(5)
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

    controller = PickPlaceController()
    # controller.move_to_pose(start_pose)
    controller.run()

