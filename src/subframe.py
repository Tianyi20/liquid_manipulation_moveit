#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from gripper_control import Gripper
import sys  
from moveit_msgs.msg import OrientationConstraint, Constraints
from shape_msgs.msg import SolidPrimitive

import tf
import tf.transformations
import numpy as np

class RobotController(object):
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        # Initialize the move group for the manipulator
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        
        #self.move_group.set_planner_id("RRTstar")

        group_names = self.robot.get_group_names()
        print("Available groups:", group_names)

        
        current_planner_id = self.move_group.get_planner_id()
        print("Current planner ID:", current_planner_id)

        self.move_group.allow_replanning(True)

        ## import the gripper class
        self.gripper = Gripper()
        ## here we define the tip_to_ee pose
        self.tip_to_ee = Pose()
        self.tip_to_ee.position.x = -0.15
        self.tip_to_ee.orientation.w = 1

    def pose_callback(self, msg):
        """Callback function that gets executed when a PoseStamped message is received."""
        rospy.loginfo("Received pose from topic 'transformed_pose'")
        self.object_pose = msg.pose  # Extract the pose from the PoseStamped message
        
        print("The pose of the target is:", self.object_pose)
        # Unregister the subscriber to only process the pose once
        self.subscriber.unregister()
        
        # Open the gripper
        self.gripper.open_gripper()
        rospy.loginfo("Open gripper!")
        
        # 2-> Move to middle parrallel position
        self.move_to_parrallel_target(self.object_pose)
        # Once the pose is received, move to the object's position
        rospy.loginfo("Successfully reached the middle state of the object target")
        rospy.loginfo("ready to go down")
        rospy.sleep(2)
        
        # 3-> Go down to target position, grab
        self.move_to_object_pose(self.object_pose)
        rospy.loginfo("Successfully reached the object target")
        self.gripper.close_gripper()
        rospy.loginfo("close gripper!")
        rospy.sleep(1)
        
        #TODO: add tommatto soup and attach it to end effector without seeing as collision
        self.add_box()
        self.attach_box()
        
        #TODO: Lift up the tomatto soup a little
        self.lift_up_soup()
        rospy.sleep(1)

        #todo: move to the fixed pouring position with constrained preventing water leakage
        success_pouring = self.move_to_pouring_position() 
        if not success_pouring:
            rospy.logwarn("Planning fail, exit all.")
            sys.exit(1)  
        rospy.sleep(1)

        #todo: only change the joint_6 to make a good pouring action
        self.pouring_water()
        rospy.sleep(1)
        
        #todo: change back joint_6 to be upward
        self.back_to_water_upward()
        rospy.sleep(1)

        #use forward kinematics to place the soup on table again
        self.place_back_to_table()
        rospy.sleep(1)
        self.gripper.open_gripper()
        rospy.loginfo("close gripper!")
        rospy.loginfo("Task completed!")
    
    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
        """
        Ensure that collision object updates are reflected in the planning scene.
        """
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
    
    ## define two collision objects: One water bottle, and the other is kettle 
    def spawn_collision_objects(self):
        # One water bottle
        water_bottle = PoseStamped()
        water_bottle.header.frame_id = "virtual_ee_link"
        water_bottle.pose.orientation.w = 1.0
        water_bottle.pose.position.z = 0 # slightly above the end effector
        self.scene.add_cylinder( name =  "water_bottle_cylinder", pose = water_bottle, height = 0.08, radius = 0.02 )
        ## wait for the object to come in
        self.wait_for_state_update(box_name = "water_bottle_cylinder")
        
        # subframe_virtual_rotation with the water bottle
        subframe_pose = PoseStamped()
        subframe_pose.header.frame_id = "virtual_ee_link"
        subframe_pose.pose.orientation.w = 1.0
        subframe_pose.pose.position.x = -0.15
        self.scene.add_sphere( name = "subframe_virtual_rotation", pose = subframe_pose, radius= 0.01)
        ## wait for the object to come in

        self.wait_for_state_update(box_name = "subframe_virtual_rotation")
        # def attach_box(self, link, name, pose=None, size=(1, 1, 1), touch_links=[]):
        
        ## now attach two objects to the end effector
        grasping_group = 'endeffector'
        touch_links =  self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(link = "virtual_ee_link", name = "water_bottle_cylinder" ,touch_links=touch_links)
        self.scene.attach_box(link = "virtual_ee_link", name = "subframe_virtual_rotation" ,touch_links=touch_links)
        
        return self.wait_for_state_update(box_name = "subframe_virtual_rotation") and self.wait_for_state_update(box_name = "water_bottle_cylinder")


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
    
    def get_current_tip_pose(self):
        """
        get the current tip pose attached to the tip
        """
        ee_pose = self.move_group.get_current_pose().pose
        print("the current ee_pose is:",self.move_group.get_current_pose())
        print("the current ee_pose is:", ee_pose)
        tip_pose_relative_ee = self.tip_to_ee
                
        # transform the ee pose to T_EE
        T_EE = tf.transformations.quaternion_matrix([ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w])
        T_EE[:3, 3] = [ee_pose.position.x, ee_pose.position.y, ee_pose.position.z]

        #  T_tip_to_ee
        T_tip_to_ee = tf.transformations.quaternion_matrix([tip_pose_relative_ee.orientation.x, tip_pose_relative_ee.orientation.y, tip_pose_relative_ee.orientation.z, tip_pose_relative_ee.orientation.w])
        T_tip_to_ee[:3, 3] = [tip_pose_relative_ee.position.x, tip_pose_relative_ee.position.y, tip_pose_relative_ee.position.z]

        T_tip = T_EE.dot( T_tip_to_ee )# equivalent to T_EE * T_tip_to_ee
        
        current_tip_pose = Pose()
    
        # Convert T_tip back to a Pose
        current_tip_pose = Pose()
        current_tip_pose.position.x = T_tip[0, 3]
        current_tip_pose.position.y = T_tip[1, 3]
        current_tip_pose.position.z = T_tip[2, 3]
    
        q = tf.transformations.quaternion_from_matrix(T_tip)
        current_tip_pose.orientation.x = q[0]
        current_tip_pose.orientation.y = q[1]
        current_tip_pose.orientation.z = q[2]
        current_tip_pose.orientation.w = q[3]

        return current_tip_pose

        
    def set_tip_pose(self, desired_tip_pose):
        """
        Set the desired pose for the tip, and calculate the necessary end-effector pose.
        """
        tip_pose_relative_ee = self.tip_to_ee
        
        # Transform the desired tip pose to a transformation matrix (T_tip_desired)
        T_tip_desired = tf.transformations.quaternion_matrix([
            desired_tip_pose.orientation.x, 
            desired_tip_pose.orientation.y, 
            desired_tip_pose.orientation.z, 
            desired_tip_pose.orientation.w
        ])
        T_tip_desired[:3, 3] = [
            desired_tip_pose.position.x, 
            desired_tip_pose.position.y, 
            desired_tip_pose.position.z
        ]

        # Transform the relative tip-to-EE pose into a matrix (T_tip_to_ee)
        T_tip_to_ee = tf.transformations.quaternion_matrix([
            tip_pose_relative_ee.orientation.x, 
            tip_pose_relative_ee.orientation.y, 
            tip_pose_relative_ee.orientation.z, 
            tip_pose_relative_ee.orientation.w
        ])
        T_tip_to_ee[:3, 3] = [
            tip_pose_relative_ee.position.x, 
            tip_pose_relative_ee.position.y, 
            tip_pose_relative_ee.position.z
        ]

        # Calculate the required end-effector pose (T_EE_required)
        T_EE_required = np.dot(T_tip_desired, np.linalg.inv(T_tip_to_ee))

        # Convert T_EE_required back to a Pose message for the end-effector
        ee_pose = Pose()
        ee_pose.position.x = T_EE_required[0, 3]
        ee_pose.position.y = T_EE_required[1, 3]
        ee_pose.position.z = T_EE_required[2, 3]

        q = tf.transformations.quaternion_from_matrix(T_EE_required)
        ee_pose.orientation.x = q[0]
        ee_pose.orientation.y = q[1]
        ee_pose.orientation.z = q[2]
        ee_pose.orientation.w = q[3]

        # Use move_group to set this as the target pose
        rospy.loginfo("now set_tip_pose")
        return self.move_to_pose(ee_pose)

    
    def move_to_pouring_position(self):
        """
        Move the robot to the fixed pouring position with a pose constraint 
        to prevent water leakage by keeping the bowl upright.
        """
        # Clear any existing constraints before setting new ones
        self.move_group.clear_path_constraints()
        wpose = self.move_group.get_current_pose().pose
        # Set the target position
        target_pose = Pose()
        target_pose.position.x = wpose.position.x 
        target_pose.position.y = wpose.position.y - 0.5
        target_pose.position.z = wpose.position.z

        # Set the target orientation to keep the bowl parallel to the desk
        target_pose.orientation.x = wpose.orientation.x
        target_pose.orientation.y = wpose.orientation.y
        target_pose.orientation.z = wpose.orientation.z
        target_pose.orientation.w = wpose.orientation.w

        # Create orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = "virtual_ee_link"  # Make sure this matches your actual end-effector link name
        orientation_constraint.header.frame_id = "base_link"  # Make sure this matches your robot's base frame
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.2 # Allowable error on the X-axis
        orientation_constraint.absolute_y_axis_tolerance = 0.2 # Allowable error on the Y-axis
        orientation_constraint.absolute_z_axis_tolerance = 0.2 # Allow larger freedom on the Z-axis
        orientation_constraint.weight = 1

        # Add the orientation constraint to path constraints
        constraints = Constraints()
        constraints.orientation_constraints.append(orientation_constraint)
        self.move_group.set_path_constraints(constraints)
        self.move_group.set_planner_id("RRTstarkConfigDefault") 
        # Increase planning time for a better chance of finding a solution
        self.move_group.set_num_planning_attempts(20)
        
        # Plan and execute the motion to the pouring position
        rospy.loginfo("Moving to pouring position with upward orientation constraint...")
        success = self.move_to_pose(target_pose)

        # Stop and clear constraints after execution
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        self.move_group.clear_path_constraints()

        if success:
            rospy.loginfo("Successfully reached the pouring position with correct orientation.")
        else:
            rospy.logwarn("Failed to reach the pouring position with correct orientation.")
        
        return success


    def rotation_plus_euler(self, euler_added, end_effector = "tip"):
        """
        maintain the current pose, but only change the euler angle
        """        
        current_pose = self.get_current_tip_pose()
        target_pose = Pose()

        current_orientation_q = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        ]
        ## convert the euler angle to be quaternion
        quaternion_added = tf.transformations.quaternion_from_euler(*euler_added)
        ## quaternion multiply realize the rotation overlap
        target_orientation_q = tf.transformations.quaternion_multiply(current_orientation_q, quaternion_added)
        
        # Set position to the current position to maintain it
        target_pose.position.x = current_pose.position.x
        target_pose.position.y = current_pose.position.y
        target_pose.position.z = current_pose.position.z
        target_pose.orientation.x = target_orientation_q[0]
        target_pose.orientation.y = target_orientation_q[1]
        target_pose.orientation.z = target_orientation_q[2]
        target_pose.orientation.w = target_orientation_q[3]

        # move to new pose
        if end_effector == "tip":
            return  self.set_tip_pose(target_pose)
        else:
            return self.move_to_pose(target_pose)
    
    def move_to_camera_position(self):
        """
        Move to a predefined camera scanning position using forward kinematics.
        """
        joint_goal = [2.8024751799, -1.57, 1.66, 1.05, 1.62, -3.07]  # Example joint goal for camera position
        return self.move_to_joint_goal(joint_goal)
    
    
    



if __name__ == '__main__':
    try:
        robot = RobotController()
        
        # Step 1: Move to the camera scan position using joint goals (forward kinematics)
        success_stage1 = robot.move_to_camera_position()
        
        if not success_stage1:
            rospy.logwarn("Failed to reach the camera scan position. Exiting program.")
            sys.exit(1)  

        ## generate two collision objects: one cylinder and one kettle
        robot.spawn_collision_objects()
        
        print("current tip pose is",robot.get_current_tip_pose())
        
        robot.rotation_plus_euler(euler_added= [0,0.5,0],end_effector = "tip")
        
        robot.rotation_plus_euler(euler_added= [0,0.5,0],end_effector = "tip")

        robot.rotation_plus_euler(euler_added= [0,-0.5,0],end_effector = "tip")

        robot.rotation_plus_euler(euler_added= [0,-0.5,0],end_effector = "tip")

        # # change the endeffector to be the cylinder's tip while transfer the cylinder_tip to the kettle's 
        #robot.change_end_effector_link(changed_end_effector_link = "subframe_virtual_rotation")
        #robot.move_group.set_end_effector_link(link_name = "subframe_virtual_rotation")
        
        #robot.rotation_plus_euler(euler_added = [5,5,5])
        # ## from now on, the moveit will try to plan with end effector link "cylinder_water_bottle/tip_bottle"
        # # 
        # euler_added = [0,50,0]        
        # robot.rotation_plus_euler(euler_added)
        # ## planning to target pose with the changed end effector
        







        
        

    except rospy.ROSInterruptException:
        pass
