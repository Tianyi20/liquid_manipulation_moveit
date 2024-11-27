#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, WrenchStamped
from gripper_control import Gripper
import sys  
from moveit_msgs.msg import OrientationConstraint, Constraints
from shape_msgs.msg import SolidPrimitive
import copy

import tf
import tf.transformations
import numpy as np


class RobotController(object):
    def __init__(self):
        # Initialize the move group for the manipulator
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        
        ## subsribe the /wrench topic
        rospy.Subscriber('/wrench', WrenchStamped, self.weight_callback)
        
        #self.move_group.set_planner_id("RRTstar")
        # use this to set the accerleration and velocity of robot
        # self.move_group.set_max_velocity_scaling_factor(0.01)
        # self.move_group.set_max_acceleration_scaling_factor(0.01)
        group_names = self.robot.get_group_names()
        print("Available groups:", group_names)
        
        setted_planning_time = self.move_group.get_planning_time()
        print("setted planning time:", setted_planning_time)

        current_planner_id = self.move_group.get_planner_id()
        print("Current planner ID:", current_planner_id)

        #self.move_group.allow_replanning(True)

        ## import the gripper class
        self.gripper = Gripper()
        ## here we define the tip_to_ee pose
        self.tip_to_ee = Pose()
        self.tip_to_ee.position.x = -0.15
        self.tip_to_ee.orientation.w = 1
        
        self.weight_loss_threshold = 10
        
        self.initial_weight = None
        self.current_weight = None
        
    def weight_callback(self, data):
        """Callback to monitor the weight on the gripper."""
        self.current_weight = data.wrench.force.y
        ##print("self.current_weight", self.current_weight)
        # Initialize the starting weight
            
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
        water_bottle.pose.orientation.x = 0
        water_bottle.pose.orientation.y = 0.70710678
        water_bottle.pose.orientation.z = 0
        water_bottle.pose.orientation.w = 0.70710678
        water_bottle.pose.position.x = -0.04 # slightly above the end effector
        self.scene.add_cylinder( name =  "water_bottle_cylinder", pose = water_bottle, height = 0.2, radius = 0.023 )
        ## wait for the object to come in
        self.wait_for_state_update(box_name = "water_bottle_cylinder")
        
        kettle_pose = PoseStamped()
        kettle_pose.header.frame_id = "base_link"
        kettle_pose.pose.position.x = -0.656
        kettle_pose.pose.position.y = -0.37858
        kettle_pose.pose.position.z = 0.12
        kettle_pose.pose.orientation.w = 1
        self.scene.add_cylinder( name =  "kettle", pose = kettle_pose, height = 0.24, radius = 0.0675 )
        ## wait for the object to come in
        self.wait_for_state_update(box_name = "kettle")
        
        # # subframe_virtual_rotation with the water bottle
        # subframe_pose = PoseStamped()
        # subframe_pose.header.frame_id = "virtual_ee_link"
        # subframe_pose.pose.orientation.w = 1.0
        # subframe_pose.pose.position.x = -0.1
        # self.scene.add_sphere( name = "subframe_virtual_rotation", pose = subframe_pose, radius= 0.01)
        # ## wait for the object to come in

        # self.wait_for_state_update(box_name = "subframe_virtual_rotation")
        # def attach_box(self, link, name, pose=None, size=(1, 1, 1), touch_links=[]):
        
        ## now attach two objects to the end effector
        grasping_group = 'endeffector'
        touch_links =  self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(link = "virtual_ee_link", name = "water_bottle_cylinder" ,touch_links=touch_links)
        # self.scene.attach_box(link = "virtual_ee_link", name = "subframe_virtual_rotation" ,touch_links=touch_links)
        
        return self.wait_for_state_update(box_name = "water_bottle_cylinder")


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
    
    def cal_tip_from_ee(self, ee_pose):
        """
        get the current tip pose attached to the tip
        """
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


    def cal_ee_from_tip(self, desired_tip_pose):
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
        #rospy.loginfo("now set_tip_pose")
        return ee_pose

    
    def move_to_pouring_position(self, target_pose ):
        """
        Move the robot to the fixed pouring position with a pose constraint 
        to prevent water leakage by keeping the bowl upright.
        """
        # Clear any existing constraints before setting new ones
        self.move_group.clear_path_constraints()
        wpose = self.move_group.get_current_pose().pose
        # Set the target position
        
        # lift up to the target position's z position
        wpose.position.z = target_pose.position.z
        
        # here we lift the cup a little firstly
        success = self.move_to_pose(copy.deepcopy(wpose))
        if success == False:
            return success
                
        # Create orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = "virtual_ee_link"  # Make sure this matches your actual end-effector link name
        orientation_constraint.header.frame_id = "base_link"  # Make sure this matches your robot's base frame
        orientation_constraint.orientation = copy.deepcopy(wpose).orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.2 # Allowable error on the X-axis
        orientation_constraint.absolute_y_axis_tolerance = 0.2 # Allowable error on the Y-axis
        orientation_constraint.absolute_z_axis_tolerance = 0.2 # Allow larger freedom on the Z-axis
        orientation_constraint.weight = 1

        # Add the orientation constraint to path constraints
        constraints = Constraints()
        #constraints.orientation_constraints.append(orientation_constraint)
        #self.move_group.set_path_constraints(constraints)
        #self.move_group.set_planner_id("RRTstarkConfigDefault") 
      
        # Plan and execute the motion to the pouring position
        rospy.loginfo("Moving to pouring position with upward orientation constraint...")
        
        wpose.position.x = wpose.position.x
        wpose.position.y = wpose.position.y - 0.5
        success = self.move_to_pose(copy.deepcopy(wpose))

        # Stop and clear constraints after execution
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        self.move_group.clear_path_constraints()
        
        # self.move_to_pose(target_pose)

        if success:
            rospy.loginfo("Successfully reached the pouring position with correct orientation.")
        else:
            rospy.logwarn("Failed to reach the pouring position with correct orientation.")
        
        return success


    def rotation_plus_euler(self, current_pose, euler_added):
        """
        maintain the current pose, but only change the euler angle
        """        
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

        return target_pose
    
    def move_to_camera_position(self):
        """
        Move to a predefined camera scanning position using forward kinematics.
        """
        joint_goal = [2.8024751799, -1.57, 1.66, 1.05, 1.62, -3.07]  # Example joint goal for camera position
        return self.move_to_joint_goal(joint_goal)
    
    def interpolate(self):
        
        if self.initial_weight is None:
            self.initial_weight = self.current_weight
            print("self.initial_weight is", self.initial_weight)


        waypoints = []
        
        wpose = self.move_group.get_current_pose().pose
        tip_position = self.cal_tip_from_ee(copy.deepcopy(wpose))
        
        tip_position.position.x -=  0.05  # First move up (z)
        tip_position.position.y -=  0.05  # and sideways (y)
        tip_position.position.z -=  0.05  # and sideways (y)

        #tip_position.position.z -=  0.05  # and sideways (y)

        #waypoints.append(copy.deepcopy(wpose))
    
        #wpose.position.y -= scale * 0.1  # Second move forward/backwards in (x)
        #wpose.position.z += scale * 0.1  # Second move forward/backwards in (x)
        #get tip position, add rotation, calculate back to ee
        
        tip_position_rotate = self.rotation_plus_euler(current_pose= copy.deepcopy(tip_position), euler_added= [0,1,0])
        wpose = self.cal_ee_from_tip(copy.deepcopy(tip_position_rotate))
        
        waypoints.append(copy.deepcopy(wpose))
        
        tip_position = self.cal_tip_from_ee(copy.deepcopy(wpose))
        tip_position.position.z -= 0.05
        tip_position_rotate = self.rotation_plus_euler(current_pose= copy.deepcopy(tip_position), euler_added= [0,1,0])
        wpose = self.cal_ee_from_tip(copy.deepcopy(tip_position_rotate))
        waypoints.append(copy.deepcopy(wpose))

        
        # tip_position = self.cal_tip_from_ee(copy.deepcopy(wpose))
        # tip_position_rotate = self.rotation_plus_euler(current_pose= tip_position, euler_added= [0,0.5,0])
        # wpose = self.cal_ee_from_tip(tip_position_rotate)
        
        # waypoints.append(copy.deepcopy(wpose))
        
        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.0001,        # eef_step
                                        0)         # jump_threshold
        print("the cartesian path fraction is", fraction)
        
        ## !! we set the execture to be asynchonize that allow to run the following program  while runing .move_group.execute the last plan
        self.move_group.execute(plan, wait=False) 
        
        self.monitor_weight_loss()
        
        rospy.loginfo("Trajectory completed or stopped due to weight loss.")
        
        return
        
    def monitor_weight_loss(self):
        """Monitors weight loss and stops the movement if threshold is reached."""
        rospy.loginfo("Here the program is in monitor weight loss")
        rate = rospy.Rate(10)  # Set the rate of monitoring
        while not rospy.is_shutdown():
            if self.current_weight is not None:
                weight_loss = self.initial_weight - self.current_weight
                print("the weight lost is:",weight_loss)
                if weight_loss >= self.weight_loss_threshold:
                    rospy.loginfo("Weight loss exceeded threshold. Stopping trajectory.")
                    self.move_group.stop()
                    return  # Exit once the trajectory is stopped
            rate.sleep()
    
    def initial_bottle_position(self):
        initial_water_bottle = PoseStamped()
        initial_water_bottle.header.frame_id = "base_link"
        initial_water_bottle.pose.orientation.w = 1
        initial_water_bottle.pose.position.x = -0.565216876368
        initial_water_bottle.pose.position.y = 0.262090677737
        initial_water_bottle.pose.position.z = 0.1

        self.scene.add_cylinder( name =  "initial_bottle", pose = initial_water_bottle, height = 0.2, radius = 0.05 )
        ## wait for the object to come in
        self.wait_for_state_update(box_name = "initial_bottle")
        
    def finish_position(self):
        """
        Move to a predefined camera scanning position using forward kinematics.
        """
        
        joint_goal = [3.58159016, -1.54217293, 2.24431889, -0.679631211, 1.65073241, -3.13583307]  # Example joint goal for camera position
        self.move_group.set_start_state_to_current_state()
        return self.move_to_joint_goal(joint_goal)
    
    def move_to_object_pose(self, object_pose):
        """
        Move to the object's position using inverse kinematics.
        object_pose: geometry_msgs/Pose
        """
        pick_pose = Pose()
        pick_pose.position.x = object_pose.position.x 
        pick_pose.position.y = object_pose.position.y
        pick_pose.position.z = object_pose.position.z
        
    ##### !!!!!!!!!!!! Orientation that be parrallel to desk
    # go_orientation.orientation.x = -0.71612s
    # go_orientation.orientation.y = 0.0175
    # go_orientation.orientation.z = 0.6976
    # go_orientation.orientation.w = -0.01
    
        pick_pose.orientation.x = -0.71612
        pick_pose.orientation.y = 0.0175
        pick_pose.orientation.z = 0.6976
        pick_pose.orientation.w = -0.01
        
        print("The target pick_pose is:",pick_pose)

        return self.move_to_pose(pick_pose)
        

class PoseSubscriber(object):
    def __init__(self):
        # Create storage for bottle and cup poses
        self.bottle_pose = Pose()
        self.cup_pose = Pose()

        # Flags to check if poses have been received
        self.bottle_pose_received = False
        self.cup_pose_received = False
        
        # Subscribe to the topics and assign callbacks
        rospy.Subscriber("transformed_cup_pose", PoseStamped, self.pose_cup_callback)
        rospy.Subscriber("transformed_bottle_pose", PoseStamped, self.pose_bottle_callback)

    def pose_bottle_callback(self, msg):
        """Callback to update the bottle pose."""
        self.bottle_pose = msg.pose
        self.bottle_pose_received = True
        rospy.loginfo("Updated bottle pose.")

    def pose_cup_callback(self, msg):
        """Callback to update the cup pose."""
        self.cup_pose = msg.pose
        self.cup_pose_received = True
        rospy.loginfo("Updated cup pose.")

    def get_bottle_pose(self):
        """Get the latest bottle pose if available, otherwise raise an exception."""
        if not self.bottle_pose_received:
            rospy.logwarn("Bottle pose has not been received yet. Returning None.")
            return None
        return self.bottle_pose

    def get_cup_pose(self):
        """Get the latest cup pose if available, otherwise raise an exception."""
        if not self.cup_pose_received:
            rospy.logwarn("Cup pose has not been received yet. Returning None.")
            return None
        return self.cup_pose
    



if __name__ == '__main__':    
    try:
        rospy.init_node('robot_controller', anonymous=True)

        robot = RobotController()

        current_pose = robot.move_group.get_current_pose().pose
        print("current_pose is:",current_pose)
    except rospy.ROSInterruptException:
        pass
