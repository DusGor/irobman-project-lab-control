#!/usr/bin/env python3
import sys
from typing import List, Union
import numpy as np

import actionlib
from irobman_project_lab_control.msg import ManipulatorControlAction, ManipulatorControlFeedback, ManipulatorControlResult
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import shape_msgs.msg
import trajectory_msgs.msg
import tf
import math
from scipy.spatial import KDTree
from gripper_controller import GripperController
from nav_msgs.msg import Odometry
import copy
import argparse

parser = argparse.ArgumentParser(
    description="irobman-project-lab-perception"
)

parser.add_argument(
    "--sim_mode", type=bool, default=True, help="Whether to configure the node for simulation or the real robot"
)

# parse the arguments
args_cli = parser.parse_args()

print(f"SIM MODE: {args_cli.sim_mode}")


class PickPlaceController:
    def __init__(self, cube_num=5) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_place_controller")
        rospy.loginfo("Pick Place Controller Started ...")

        self.cube_size = 0.045  # Added in class to use in calculations later
        self.cube_num = cube_num

        self.CUBE_HOVER_Z = 0.25
        self.CUBE_GRASP_Z = self.cube_size + 0.08
        self.GRASP_TOLERANCE = 0.10

        self.robot = moveit_commander.RobotCommander()  # type: ignore
        self.scene = moveit_commander.PlanningSceneInterface()  # type: ignore
        group_name = "panda_arm"
        self.move_group: moveit_commander.MoveGroupCommander.MoveGroupCommander = moveit_commander.MoveGroupCommander(group_name)  # type: ignore
        #  self.move_group.set_planner_id("RRTstar")  # Better for tight spaces
        self.move_group.set_planning_time(10)
        #  self.move_group.set_goal_tolerance(0.01)
        self.move_group.allow_replanning(True)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_start_state_to_current_state()

        self.server = actionlib.SimpleActionServer("manipulator_control", ManipulatorControlAction, self.execute, False)
        self.server.start()

        self.init_joint_angles = [
                -0.04753676322737352,
                -0.8375225578454684,
                -0.002787040147540649,
                -2.6292071353672037,
                -0.002262416709746875,
                1.9040078309531414,
                0.7482308930369274,
            ]

        self.tau = 2 * np.pi  # type: ignore

        self._planning_frame = self.robot.get_planning_frame()

        print(f"Planning frame: {self._planning_frame}")

        # Register Cubes
        self.cube_poses = {}
        self.cube_names = []

        # TODO: Subscribe to PoseArray
        if args_cli.sim_mode:
            for i in range(cube_num):
                rospy.loginfo(f"Subscribing to Topic /cube_{i}_odom")
                rospy.Subscriber(
                    f"/cube_{i}_odom",
                    data_class=Odometry,
                    callback=self._get_cube_info,
                    queue_size=10,
                )
        else:
            rospy.Subscriber(
                "cube_pose",
                data_class=geometry_msgs.msg.PoseArray,
                callback=self._get_cube_info,
                queue_size=10,
            )

        self._add_collision_objects()
        rospy.sleep(5)  # Wait for cube poses

        self.gripper = GripperController()

    def _get_cube_poses(self) -> dict:
        return self.cube_poses

    def _get_cube_info(self, pose_array: geometry_msgs.msg.PoseArray):
        # DONE: Change this to work with Pose Array
        # self.cube_poses should be a list with all cube poses, of Datatype geometry_msgs.msg.Pose
        # Define own cube names, based on number of cubes in PoseArray
        if args_cli.sim_mode:
            cube_name = pose_array.child_frame_id  # pose_array here is of type Odometry
            self.cube_poses[cube_name] = pose_array.pose.pose
        else:
            for i, pose in enumerate(pose_array.poses):
                # print(pose)
                # cube_name = odo.child_frame_id
                cube_name = f"cube_{i}"
                self.cube_poses[cube_name] = pose

    def _open_gripper(self):
        rospy.loginfo("Opening Gripper")
        return self.gripper.set_width(0.08)

    def _close_gripper(self):
        rospy.loginfo("Closing Gripper")
        return self.gripper.grasp()

    def _create_collision_object(
        self,
        id,
        dimensions,
        pose: Union[geometry_msgs.msg.Pose, Odometry],
    ):
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
            if "cube" in obj or "table" in obj or "wall" in obj:
                self.scene.remove_world_object(obj)

        # * next, we want to create a collision object for the table
        table_size = (0.81, 1.49, 0.787)  # From launch config
        table_pose = geometry_msgs.msg.Pose()
        table_pose.position.x = 0.495
        table_pose.position.y = 0.0
        table_pose.position.z = -0.3935

        table = self._create_collision_object(
            id="table", dimensions=table_size, pose=table_pose
        )
        self.scene.add_object(table)

        # * Now, we cant to create collision objects for the walls behind

        wall_size = (1.5, 0.1, 2)
        wall_pose = geometry_msgs.msg.Pose()

        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.pi / 4)

        wall_pose.orientation.x = quaternion[0]
        wall_pose.orientation.y = quaternion[1]
        wall_pose.orientation.z = quaternion[2]
        wall_pose.orientation.w = quaternion[3]

        wall_pose.position.x = -0.5
        wall_pose.position.y = 0.5
        wall_pose.position.z = 1
        wall = self._create_collision_object(
            id="wall1", dimensions=wall_size, pose=wall_pose
        )

        #  self.scene.add_object(wall)

        quaternion = tf.transformations.quaternion_from_euler(0, 0, -math.pi / 4)

        wall_pose.orientation.x = quaternion[0]
        wall_pose.orientation.y = quaternion[1]
        wall_pose.orientation.z = quaternion[2]
        wall_pose.orientation.w = quaternion[3]

        wall_pose.position.x = -0.5
        wall_pose.position.y = -0.5
        wall_pose.position.z = 1

        wall = self._create_collision_object(
            id="wall2", dimensions=wall_size, pose=wall_pose
        )

        #  self.scene.add_object(wall)

        # * Iterate over all cubes and add them as collision objects
        for cube_name, cube_pose in self._get_cube_poses().items():
            if cube_pose is not None:
                cube = self._create_collision_object(
                    id=cube_name,
                    dimensions=[self.cube_size, self.cube_size, self.cube_size],
                    pose=cube_pose,
                )
                self.scene.add_object(cube)

    def _get_cube_order(self) -> list:

        cube_order = []
        cube_poses = self._get_cube_poses()
        points = []
        labels = []

        for key, value in cube_poses.items():
            value.position.z = 0  # We dont care about height

            labels.append(key)
            points.append((value.position.x, value.position.y))

        # print(points)
        tree = KDTree(points)
        distances, _ = tree.query(points, k=2)  # Get nearest neighbors of points
        nearest_distances = distances[
            :, 1
        ]  # Get only nearest neighbor distances (not to self)
        sorted_indices = np.argsort(
            -nearest_distances
        )  # Sort indices based on nearest neighbor distance
        # print(nearest_distances)
        # Sort labels by rank (sorted_indices gives the order based on distance)
        cube_order = [labels[idx] for idx in sorted_indices]
        
        return cube_order

    def _get_cube_location(self, cube):
        return self.cube_poses[cube]

    def _get_cube_grasp(self, cube):

        nullify_angles = [0.00, 1.57, 3.14, -1.57, -3.14]

        cube_loc = self._get_cube_location(cube)
        cube_quat = [
            cube_loc.orientation.x,
            cube_loc.orientation.y,
            cube_loc.orientation.z,
            cube_loc.orientation.w,
        ]

        cube_rpy = list(tf.transformations.euler_from_quaternion(cube_quat))

        grasp_yaw = cube_rpy[2]
        grasp_yaw += (math.pi/4)
        
        quaternion = tf.transformations.quaternion_from_euler(
            0, 
            math.pi, # point down 
            grasp_yaw
        )
        print(f"Grasping {tf.transformations.euler_from_quaternion(quaternion)}")
        cube_loc.orientation.x = quaternion[0]
        cube_loc.orientation.y = quaternion[1]
        cube_loc.orientation.z = quaternion[2]
        cube_loc.orientation.w = quaternion[3]

        grasp = cube_loc
        return grasp


    def _get_ideal_tower_location(self) -> geometry_msgs.msg.Pose:

        return self._get_cube_poses()[self._get_cube_order()[0]]

    def move_to_pose(self, pose: geometry_msgs.msg.Pose) -> bool:
        # TODO: Is there some way to do proper collision-checking? Right now, Path planning is just timing out when no solution is found
        rospy.loginfo(f"Target Pose: {pose}")
        self.move_group.set_pose_target(pose, end_effector_link="panda_link8")
        result: bool = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        return result

    def follow_trajectory(self, goal_pose: geometry_msgs.msg.Pose) -> bool:

        return True

    def _pick(self, grasp_pose: geometry_msgs.msg.Pose) -> bool:
        # TODO: Implement Proper Trajectory Generation

        for (
            retries
        ) in range(  # We should make this loop detect if the cube is picked up using the camera
            6
        ):  # As of now, the planning fails sometimes (ABORTED: TIMED_OUT). These nested if-statements make sure that it is restarted until a solution is found. TODO: Debug this properly. Also, when we are restarting the Positions dont really work anymore
            rospy.loginfo(f"Pick Attempt {retries+1}")
            # Go Above Cube
            pose = geometry_msgs.msg.Pose()
            cube_pose = copy.deepcopy(grasp_pose)
            pose.orientation = cube_pose.orientation
            pose.position.x = cube_pose.position.x
            pose.position.y = cube_pose.position.y
            pose.position.z = self.CUBE_HOVER_Z + 0.1

            rospy.loginfo(f"Executing Pick Action for Pose {cube_pose} ...")


            self._open_gripper()
            rospy.loginfo(f"Moving Above Cube ...")

            if self.move_to_pose(pose):

                # Move down to cube
                rospy.loginfo(f"Moving grip to cube ...")

                pose.position.z = self.CUBE_GRASP_Z

                if self.move_to_pose(pose):

                    self._close_gripper()

                    # # Close Grasp
                    # if (
                    #     not self._close_gripper()
                    # ):  # If grasp uncessful, rotate grasp a bit and try again
                    #     rospy.loginfo("Reorienting Cube ...")
                    #     cube_quat = [
                    #         pose.orientation.x,
                    #         pose.orientation.y,
                    #         pose.orientation.z,
                    #         pose.orientation.w,
                    #     ]
                    #     cube_rpy = list(
                    #         tf.transformations.euler_from_quaternion(cube_quat)
                    #     )
                    #     cube_rpy[2] -= math.pi / 4
                    #     quaternion = tf.transformations.quaternion_from_euler(
                    #         0, math.pi, cube_rpy[2]
                    #     )

                    #     pose.orientation.x = quaternion[0]
                    #     pose.orientation.y = quaternion[1]
                    #     pose.orientation.z = quaternion[2]
                    #     pose.orientation.w = quaternion[3]

                    #     continue

                    # else:
                    rospy.loginfo(f"Picking up cube ...")

                    # Go back up :)
                    pose.position.z = self.CUBE_HOVER_Z + 0.1

                    if self.move_to_pose(pose):
                        return True

        return False

    def _place(self, pose: geometry_msgs.msg.Pose, name) -> bool:

        rospy.loginfo(f"Placing object at pose {pose}")

        current_pose = self.move_group.get_current_pose().pose
        place_pose = geometry_msgs.msg.Pose()
        place_pose.position = pose.position

        place_pose.position.z += 0.05  # Hover first

        for retries in range(
            5
        ):  # We should make this loop detect if the cube is placed correctly using the camera
            if self.move_to_pose(place_pose):
                place_pose.position.z -= 0.05
                if self.move_to_pose(place_pose):
                    self._open_gripper()
                    rospy.sleep(1)
                    place_pose.position.z += self.CUBE_HOVER_Z
                    self.move_to_pose(place_pose)
                return True

        return False

    def _build_tower(
        self,
    ) -> bool:
        tower_pose = geometry_msgs.msg.Pose()
        tower_pose.position.z = 0
        # print(cube_grasps)
        i = 0

        tower_pose.position.x = 0.5
        tower_pose.position.y = 0.3

        tower_quat = tf.transformations.quaternion_from_euler(0, math.pi, 0)
        tower_pose.orientation.x = tower_quat[0]
        tower_pose.orientation.y = tower_quat[1]
        tower_pose.orientation.z = tower_quat[2]
        tower_pose.orientation.w = tower_quat[3]
        
        for cube_name in self._get_cube_order():
            i += 1

            rospy.loginfo(f"Pick&Place for {cube_name}")
            self._pick(self._get_cube_grasp(cube_name))

            tower_pose.position.z = self.CUBE_GRASP_Z + ((i - 1) * (self.cube_size))
            rospy.loginfo("Initializing Place...")
            self._place(tower_pose, cube_name)

        return True

    def run(self):
        self._open_gripper()

        if args_cli.sim_mode:
            for i in range(self.cube_num):
                name = "cube_" + str(self.cube_num)
        else:
            for i, _ in enumerate(self.cube_poses):
                # for i in range(self.cube_num):
                name = f"cube_{i}"
                self.scene.remove_attached_object("panda_link8", name=name)

        rate = rospy.Rate(10)
        pose_goal = self.move_group.get_current_pose().pose

        joint_values = self.move_group.get_current_joint_values()


        while not rospy.is_shutdown():
            self._build_tower()

            self.move_group.go(init_joint_angles, wait=True)
            break


    def execute(self, goal):
        rospy.loginfo(f"Received command: {goal.command}")

        feedback = ManipulatorControlFeedback()
        result = ManipulatorControlResult()

        if goal.command == "go_to_overview":
            success = self.move_group.go(self.init_joint_angles, wait=True)
            print(success)
            result.success = success
            if result:
                feedback.feedback = "Reached Overview."
            else:
                feedback.feedback = "Failed to reach overview!"
        elif goal.command == "scan_cube":
            # TODO: DO SOMETHING
            result.success = True
            feedback.feedback = f"Reached cube pose: {goal.target_pose}"
        elif goal.command == "pick":
            # TODO: DO SOMETHING
            result.success = True
            feedback.feedback = "Object picked."
        elif goal.command == "place":
            # TODO: DO SOMETHING
            result.success = True
            feedback.feedback = "Object palced."
        else:
            result.success = False
            feedback.feedback = "Unknown command."
        self.server.publish_feedback(feedback)
        self.server.set_succeeded(result)


if __name__ == "__main__":
    controller = PickPlaceController()
    controller.run()

# TODO: Fix Pick&Place, Pick greift irgendwie leicht daneben.
# TODO: Intelligenteres Pick, schauen ob Grasp mit anderen Cubes kollidiert
