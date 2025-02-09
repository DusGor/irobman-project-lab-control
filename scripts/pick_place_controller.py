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

from gripper_controller import GripperController
from nav_msgs.msg import Odometry


class PickPlaceController:
    def __init__(self,  cube_name: str) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_place_controller")
        rospy.loginfo("Pick Place Controller Started ...")
        self.cube_size = 0.045 # Added in class to use in calculations later
        self.cube_name = cube_name

        self.robot = moveit_commander.RobotCommander() # type: ignore
        self.scene = moveit_commander.PlanningSceneInterface() # type: ignore
        group_name = "panda_arm"
        self.move_group: moveit_commander.MoveGroupCommander.MoveGroupCommander = moveit_commander.MoveGroupCommander(group_name) # type: ignore
        self.move_group.set_planner_id("RRTstar") # Better for tight spaces
        self.move_group.set_planning_time(10) 
        self.move_group.set_goal_tolerance(0.01)
        self.move_group.allow_replanning(True)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_start_state_to_current_state()

        self.tau = 2 * np.pi # type: ignore

        self._planning_frame = self.robot.get_planning_frame()

        print(f"Planning frame: {self._planning_frame}")

        rospy.Subscriber(f"/{self.cube_name}_odom", data_class=Odometry, callback=self._get_cube_info, queue_size=10)

        self.gripper_cmd_pub = rospy.Publisher("/panda_gripper/cmd", data_class=std_msgs.msg.String, queue_size=10)

        self.cube_pose: Union[geometry_msgs.msg.Pose, None] = None

        rospy.sleep(2)

        self._add_collision_objects()
        self.gripper = GripperController()

    def _get_cube_poses(self) -> List[Odometry]:
        return [self.cube_pose] # type: ignore

    def _get_cube_info(self, odo: Odometry):
        self.cube_pose = odo.pose.pose
    
    def _open_gripper(self):
        rospy.loginfo("Opening Gripper")
        self.gripper.set_width(0.08)

    
    def _close_gripper(self):
        rospy.loginfo("Closing Gripper")
        self.gripper.grasp()

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
        # * First we want to clear the scene of previous collision objects
        collision_objects = self.scene.get_known_object_names()

        for obj in collision_objects:
            if 'cube' in obj or 'table' in obj:
                self.scene.remove_world_object(obj)

        # * next, we want to create a collision object for the table
        table_size = (0.81, 1.49, 0.787) # From launch config
        table_pose = geometry_msgs.msg.Pose()
        table_pose.position.x = 0.495
        table_pose.position.y = 0.0
        table_pose.position.z = -0.3935
        table = self._create_collision_object(id='table',
                                        dimensions=table_size,
                                        pose=table_pose)
        self.scene.add_object(table)


        # * Iterate over all cubes and add them as collision objects
        for i, cube_pose in enumerate(self._get_cube_poses()):
            if cube_pose is not None:
                cube = self._create_collision_object(id=self.cube_name, dimensions=[self.cube_size, self.cube_size, self.cube_size], pose=cube_pose,)
                self.scene.add_object(cube)

        

    def move_to_pose(self, pose: geometry_msgs.msg.Pose) -> bool:
        # TODO: Is there some way to do proper collision-checking? Right now, Path planning is just timing out when no solution is found
        rospy.loginfo(f"Target Pose: {pose}")
        self.move_group.set_pose_target(pose, end_effector_link="panda_link8")
        result: bool = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        return result

    def _pick(self, cube_num) -> bool:
        CUBE_HOVER_Z = 0.4
        CUBE_GRASP_Z = self.cube_size + 0.058

        if (cube_pose := self._get_cube_poses()[cube_num]) is not None:

            rospy.loginfo(f"Executing Pick Action for cube_{cube_num} ...")

            rospy.loginfo(f"Moving above cube_{cube_num} ...")

            # Go Above Cube
            print("Cube Pose: ", cube_pose.position)
            pose = geometry_msgs.msg.Pose()
            pose.position.x = self.cube_pose.position.x
            pose.position.y = self.cube_pose.position.y
            pose.position.z = self.cube_pose.position.z + CUBE_HOVER_Z

            
            cube_quat = [self.cube_pose.orientation.x, self.cube_pose.orientation.y, self.cube_pose.orientation.z, self.cube_pose.orientation.w]
            cube_rpy = tf.transformations.euler_from_quaternion(cube_quat)
            
            quaternion = tf.transformations.quaternion_from_euler(0, math.pi, cube_rpy[2] - math.pi/4)

            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            for retries in range (5): # As of now, the planning fails sometimes (ABORTED: TIMED_OUT). These nested if-statements make sure that it is restarted until a solution is found. TODO: Debug this properly 
                if self.move_to_pose(pose): 

                    # Open Grasp
                    self._open_gripper()

                    # Move down to cube 
                    rospy.loginfo(f"Moving grip to cube_{cube_num} ...")

                    pose.position.z = self.cube_pose.position.z + CUBE_GRASP_Z

                    if self.move_to_pose(pose):


                        # Close Grasp
                        self._close_gripper()
                        rospy.loginfo(f"Picking up cube_{cube_num} ...")

                        # Go back up :)
                        pose.position.z = self.cube_pose.position.z + CUBE_HOVER_Z

                        if self.move_to_pose(pose):
                            return True

        return False

    def _place(self, cube_num) -> bool:
        
        rospy.loginfo(f"Executing Place Action...")
        pose = geometry_msgs.msg.Pose()
        cube_pose = self._get_cube_poses[cube_num]
        if cube_num == 0:
            
            pose.position.x = 0.4
            pose.position.y = 0.4
            pose.position.z = cube_pose.position.z

        
        
        
        
        # # Just place the cube in the exact same spot on the other table
        # place_location.place_pose.pose.position = cube_pose.position
        # place_location.place_pose.pose.position.x = cube_pose.position.x # Table is on mirrored x axis

        # place_location.place_pose.pose.orientation = cube_pose.orientation

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
        
        self.move_group.set_support_surface_name("table")
        return self.move_group.place(self.cube_name, place_location)
        
        return False

    def _build_tower(self) -> bool:
        return False

    def run(self):
        rate = rospy.Rate(10)
        pose_goal = self.move_group.get_current_pose().pose
        print(pose_goal)  # Print the current pose to debug
        print(self.move_group.get_current_joint_values())
        while not rospy.is_shutdown():
            # Move to overview pose to capture table with viewpoint
            # overview_pose = geometry_msgs.msg.Pose()
            # overview_pose.position.x = 0.4
            # overview_pose.position.y = 0.0
            # overview_pose.position.z = 0.7

            # quat: np.ndarray = tf.transformations.quaternion_from_euler(- self.tau / 16, self.tau / 2 - self.tau / 16, - self.tau / 8)

            # overview_pose.orientation.x = quat[0]
            # overview_pose.orientation.y = quat[1]
            # overview_pose.orientation.z = quat[2]
            # overview_pose.orientation.w = quat[3]


            init_joint_angles = [0.00013243881315272432, -0.7839791476933904, -1.2629424535504086e-05, -2.358711079936866, -2.7162019504700652e-05, 1.5713224572373559, 0.7854116670960147,]

            self.move_group.go(init_joint_angles, wait=True)

            # overview_pose = geometry_msgs.msg.Pose()
            # overview_pose.position.x = 0.3
            # overview_pose.position.y = 0.0
            # overview_pose.position.z = 0.7

            # euler = tf.transformations.euler_from_quaternion(np.array([0.9238, -0.3827, 0.0, 0.0]))

            # # quat: np.ndarray = tf.transformations.quaternion_from_euler(- self.tau / 16, self.tau / 2 - self.tau / 16, - self.tau / 8)
            # quat: np.ndarray = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])

            # overview_pose.orientation.x = quat[0]
            # overview_pose.orientation.y = quat[1]
            # overview_pose.orientation.z = quat[2]
            # overview_pose.orientation.w = quat[3]
            # if self.move_to_pose(overview_pose):
            #     rospy.sleep(5)

            # Wait for cube_pose to be available

            if self._pick(0):
                rospy.sleep(5)
                # if self._place():
                #     rospy.sleep(5)
                print("SUCCESS!")
                break
            print(f"FAILED!")
            break
        self._open_gripper()
        self.scene.remove_attached_object("panda_link8", name=self.cube_name)


if __name__ == "__main__":
    controller = PickPlaceController(cube_name="cube_0")
    controller.run()

