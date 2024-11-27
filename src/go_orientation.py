#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose

class RobotController(object):
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        # Initialize the move group for the manipulator
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
       

    def move_to_joint_goal(self, joint_goal):
        """
        Move the robot to a specific joint configuration.
        joint_goal: list of joint angles [joint_1, joint_2, ..., joint_n]
        """
        # Set the joint goal
        self.move_group.set_joint_value_target(joint_goal)

        # Plan and execute the motion to the target joint values
        rospy.loginfo("Planning to joint goal...")
        success = self.move_group.go(wait=True)
        
        # Stop the robot after planning
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Check if the goal was reached
        if success:
            rospy.loginfo("Reached the joint goal!")
        else:
            rospy.logwarn("Failed to reach the joint goal.")
        return success

    def move_to_pose(self, target_pose):
        """
        Move the robot to a specific pose using inverse kinematics.
        target_pose: geometry_msgs/Pose
        """
        # Set the pose target
        self.move_group.set_pose_target(target_pose)

        # Plan and execute the motion to the target pose
        rospy.loginfo("Planning to pose goal...")
        success = self.move_group.go(wait=True)

        # Stop the robot after planning
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Check if the goal was reached
        if success:
            rospy.loginfo("Reached the pose goal!")
        else:
            rospy.logwarn("Failed to reach the pose goal.")
        return success

    def move_to_object_pose(self, object_pose):
        """
        Move to the object's position using inverse kinematics.
        object_pose: geometry_msgs/Pose
        """
        pick_pose = Pose()
        
        pick_pose.position.x = object_pose.position.x
        pick_pose.position.y = object_pose.position.y
        pick_pose.position.z = object_pose.position.z
        pick_pose.orientation.x = 0.97576
        pick_pose.orientation.y = -0.048332
        pick_pose.orientation.z = -0.21
        pick_pose.orientation.w = -0.013
        
        print("The target pick_pose is:",pick_pose)

        return self.move_to_pose(pick_pose)
    
    def move_to_parrallel_target(self, object_pose):
        """
        Move the robot to position that X,Y same to target, but z as the robot current pose
        """        
        current_pose = self.move_group.get_current_pose().pose
        middle_use_pose = Pose()
        middle_use_pose.position.x = object_pose.position.x
        middle_use_pose.position.y = object_pose.position.y
        middle_use_pose.position.z = current_pose.position.z
        ##This orientation can keep the tool be upward and pointing outside the table
        middle_use_pose.orientation.x = 0.97576
        middle_use_pose.orientation.y = -0.048332
        middle_use_pose.orientation.z = -0.21
        middle_use_pose.orientation.w = -0.013
        print("The target middle use between the pick_pose is:",middle_use_pose)

        return self.move_to_pose(middle_use_pose)
        
    def move_to_camera_position(self):
        """
        Move to a predefined camera scanning position using forward kinematics.
        """
        joint_goal = [3.14, -1.57, 1.66, 1.05, 1.62, -3.07]  # Example joint goal for camera position
        return self.move_to_joint_goal(joint_goal)

if __name__ == '__main__':
    
    robot = RobotController()

    # Step 1: Move to the camera scan position using joint goals (forward kinematics)
    robot.move_to_camera_position()
# x: -0.571212573369
#   y: -0.0661376738856
#   z: 0.00461096150579
# orientation: 
#   x: 0.97576
#   y: -0.048332
#   z: -0.21
#   w: -0.013)

    rospy.sleep(3)
    go_orientation = Pose()
    go_orientation.position.x = -0.571212573369
    go_orientation.position.y = -0.0661376738856
    go_orientation.position.z = 0.2
    #### !!!!!!!!!!!! Orientation that be parrallel to desk
    go_orientation.orientation.x = -0.71612
    go_orientation.orientation.y = 0.0175
    go_orientation.orientation.z = 0.6976
    go_orientation.orientation.w = -0.01
    
    # virtual_ee_link
    # rviz.Orientation.X = - go_orientation.orientation.x
    # rviz.Orientation.Y = - go_orientation.orientation.Y
    # rviz.Orientation.Z = - go_orientation.orientation.Z
    # rviz.Orientation.W = - go_orientation.orientation.W

        #     self.grasp_pose.orientation.x = -0.719093153708
        # self.grasp_pose.orientation.y = -0.694812369689
        # self.grasp_pose.orientation.z = 0.00206665742698
        # self.grasp_pose.orientation.w = 0.0116848682052
        
        
    robot.move_to_pose(go_orientation)
            

