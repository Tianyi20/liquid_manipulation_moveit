#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import sys

class RobotCollisionAvoidance:
    def __init__(self):
        # Initialize the node and MoveIt! components
        rospy.init_node('collision_avoidance_node')
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")

        # Assign a name for the collision object
        self.box_name = "box"

        # Wait for the scene to be updated
        rospy.sleep(2)

    def add_box(self, timeout=4):
        """
        Create and add a box as a collision object in the MoveIt! planning scene.
        """
        # Define the box pose relative to a frame (e.g., the robot's base frame)
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"  # Replace with your robot's base frame
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = -0.6
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.2  # Position it above the end-effector

        # Define box size (0.1m x 0.1m x 0.1m)
        box_name = self.box_name
        self.scene.add_box(box_name, box_pose, size=(0.2, 0.1, 0.3))
        rospy.loginfo("Added box to the scene.")

        # Ensure the update is successful
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        """
        Ensure that collision object updates are reflected in the planning scene.
        """
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Check if the box is attached to the robot
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Check if the box is known in the scene
            is_known = box_name in scene.get_known_object_names()

            # Verify if the object is in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def move_to_goal_with_avoidance(self, target_pose):
        """
        Move the robot to a specified target pose while avoiding collisions.
        """
        # Ensure the scene is updated before moving
        rospy.sleep(1)

        # Set the target pose for the move group
        self.move_group.set_pose_target(target_pose)

        # Enable collision avoidance with a reasonable planning time
        self.move_group.set_planning_time(10.0)
        success = self.move_group.go(wait=True)

        # Stop the robot and clear the target pose
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Log success or failure
        if success:
            rospy.loginfo("Successfully reached the goal pose with collision avoidance.")
        else:
            rospy.logwarn("Failed to reach the goal pose with collision avoidance.")
        return success

if __name__ == '__main__':
    try:
        # Initialize the collision avoidance node
        robot = RobotCollisionAvoidance()
        
        # Step 2: Go to the stage1
        target_pose_1 = PoseStamped()
        target_pose_1.header.frame_id = robot.robot.get_planning_frame()
        target_pose_1.pose.position.x = -0.571212573369
        target_pose_1.pose.position.y = -0.4
        target_pose_1.pose.position.z = 0.1
        target_pose_1.pose.orientation.x =  -0.719093153708
        target_pose_1.pose.orientation.y = -0.694812369689
        target_pose_1.pose.orientation.z = 0.00206665742698
        target_pose_1.pose.orientation.w = 0.0116848682052


        
        # Step 2: go to the stage2
        target_pose_2 = PoseStamped()
        target_pose_2.header.frame_id = robot.robot.get_planning_frame()
        target_pose_2.pose.position.x = -0.571212573369
        target_pose_2.pose.position.y = 0.2
        target_pose_2.pose.position.z = 0.1
        target_pose_2.pose.orientation.x = -0.719093153708
        target_pose_2.pose.orientation.y = -0.694812369689
        target_pose_2.pose.orientation.z = 0.00206665742698
        target_pose_2.pose.orientation.w = 0.0116848682052
        
        # Step 3: Move to the target pose while avoiding collisions
        robot.move_to_goal_with_avoidance(target_pose_1.pose)
        
        # Step 1: Add a box as a collision object to the scene
        robot.add_box()
        
        while not rospy.is_shutdown():
            print("in the iteration")
            robot.move_to_goal_with_avoidance(target_pose_1.pose)
            rospy.sleep(3)
            robot.move_to_goal_with_avoidance(target_pose_2.pose)
        
    except rospy.ROSInterruptException:
        pass
