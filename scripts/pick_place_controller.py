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
import tf
import math

from nav_msgs.msg import Odometry


class PickPlaceController:
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_place_controller")
        rospy.loginfo("Pick Place Controller Started ...")
        self.cube_size  = 0.045 # Added in class to use in calculations later

        self.robot = moveit_commander.RobotCommander() # type: ignore
        self.scene = moveit_commander.PlanningSceneInterface() # type: ignore
        group_name = "panda_arm"
        self.move_group: moveit_commander.MoveGroupCommander.MoveGroupCommander = moveit_commander.MoveGroupCommander(group_name) # type: ignore
        self.move_group.set_planning_time(15)
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
    
    def _open_gripper(self, pre_grasp_posture: trajectory_msgs.msg.JointTrajectory):
        pre_grasp_posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        pre_grasp_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
        pre_grasp_posture.points[0].positions = [0.04, 0.04]
        #pre_grasp_posture.points[0].time_from_start = rospy.Duration(nsecs=int(5e8)) # type: ignore
    
    def _closed_gripper(self, grasp_posture: trajectory_msgs.msg.JointTrajectory):
        grasp_posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        grasp_posture.points = [trajectory_msgs.msg.JointTrajectoryPoint()]
        grasp_posture.points[0].positions = [0.00, 0.00]
        #grasp_posture.points[0].time_from_start = rospy.Duration(nsecs=int(5e8)) # type: ignore

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


        table_size = (0.81, 1.49, 0.787)


        table1_pose = geometry_msgs.msg.Pose()
        table1_pose.position.x = 0.495
        table1_pose.position.y = 0.0
        table1_pose.position.z = -0.3935

        table1 = self._create_collision_object(id='table1',
                                        dimensions=table_size,
                                        pose=table1_pose)
        self.scene.add_object(table1)


        table2_pose = geometry_msgs.msg.Pose()
        table2_pose.position.x = -0.495
        table2_pose.position.y = 0.0
        table2_pose.position.z = -0.3935

        table2 = self._create_collision_object(id='table2',
                                        dimensions=table_size,
                                        pose=table2_pose)
        self.scene.add_object(table2)



        for i, cube_pose in enumerate(self._get_cube_poses()):
            if cube_pose is not None:
                msg = f"Creating collision object for cube {i} at pos {cube_pose}"
                rospy.loginfo(msg)
                cube = self._create_collision_object(id=f'cube_{i}', dimensions=[self.cube_size, self.cube_size, self.cube_size], pose=cube_pose)
                self.scene.add_object(cube)

        

    def move_to_pose(self, pose: geometry_msgs.msg.Pose):
        rospy.loginfo(f"Target Pose: {pose}")
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()


    def _pick(self):

        if (cube_pose := self._get_cube_poses()[0]) is not None:

            rospy.loginfo(f"Planning Grasp for cube ...")

            # * Set grasp
            grasp = moveit_msgs.msg.Grasp()
            grasp.grasp_pose.header.frame_id = self._planning_frame
            grasp_pose: geometry_msgs.msg.Pose = grasp.grasp_pose.pose

            
            # Calculate Cube and Grasp Orientation
            cube_quat = [cube_pose.orientation.x, cube_pose.orientation.y, cube_pose.orientation.z, cube_pose.orientation.w]
            cube_rpy = tf.transformations.euler_from_quaternion(cube_quat)
            
            quaternion = tf.transformations.quaternion_from_euler(0, math.pi, cube_rpy[2] - math.pi/4) # Orientation has to be pitch=pi to point downwards and cube yaw - pi/4
            grasp_pose.orientation.x = quaternion[0]
            grasp_pose.orientation.y = quaternion[1]
            grasp_pose.orientation.z = quaternion[2]
            grasp_pose.orientation.w = quaternion[3]

            grasp_pose.position.x = cube_pose.position.x
            grasp_pose.position.y = cube_pose.position.y

            # Grasp Distance
            grasp_pose.position.z = cube_pose.position.z + self.cube_size + 0.058 #0.058 is half the distance between link 8 and the end of the EE

            # * Setting pre-grasp approach
            grasp.pre_grasp_approach.direction.header.frame_id = self._planning_frame
            # Direction is set as negative z axis as we are approaching the object in negative z direction
            grasp.pre_grasp_approach.direction.vector.z = -1.0
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

            print(f"::: Trying to grasp Cube at {cube_pose}")
            print(f"::: Placing Grasp at {grasp_pose}")

            self.move_group.set_support_surface_name("table1")

            self.move_group.pick("cube_0", grasp)

    def _place(self):
        
        if (cube_pose := self._get_cube_poses()[0]) is not None:
            place_location = moveit_msgs.msg.PlaceLocation()
            place_location.place_pose.header.frame_id = self._planning_frame

            # Just place the cube in the exact same spot on the other table
            place_location.place_pose.pose.position = cube_pose.position
            place_location.place_pose.pose.position.x = -cube_pose.position.x # Table is on mirrored x axis

            place_location.place_pose.pose.orientation = cube_pose.orientation

            # Setting Pre-Place Approach
            place_location.pre_place_approach.direction.header.frame_id = self._planning_frame
            # Direction is set as negative z axis
            place_location.pre_place_approach.direction.vector.z = -1.0
            place_location.pre_place_approach.min_distance = 0.095
            place_location.pre_place_approach.desired_distance = 0.115

            # Setting Post-Place Approach
            place_location.post_place_retreat.direction.header.frame_id = self._planning_frame
            # Direction is set as positive z axis
            place_location.post_place_retreat.direction.vector.z = 1.0
            place_location.post_place_retreat.min_distance = 0.1
            place_location.post_place_retreat.desired_distance = 0.25

            # Setting posture of ee after place
            self._open_gripper(place_location.post_place_posture)
            
            self.move_group.set_support_surface_name("table2")
            self.move_group.place("cube_0", place_location)


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Wait for cube_pose to be available
            if self.cube_pose:
                self._pick()
                rospy.sleep(5)
                self._place()
                rospy.sleep(5)
                break



if __name__ == "__main__":
    controller = PickPlaceController()
    controller.run()

