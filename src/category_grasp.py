#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, WrenchStamped, Vector3
from gripper_control import Gripper
import sys  
from moveit_msgs.msg import OrientationConstraint, Constraints
from shape_msgs.msg import SolidPrimitive
import copy

import tf
import tf.transformations
import numpy as np

from ultilities import offset_pose_optimiser


class RobotController(object):
    def __init__(self):
        # Initialize the move group for the manipulator
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        
        ## subsribe the /wrench topic
        rospy.Subscriber('/wrench', WrenchStamped, self.weight_callback)
        
        self.move_group.set_planner_id("RRTstar")
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
        self.tip_to_ee.position.x = -0.13
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

    
    def constrained_move_to_pose(self, target_pose):
        """
        Move the robot to the fixed pouring position with a pose constraint 
        to prevent water leakage by keeping the bowl upright.
        """
        ## Clear any existing constraints before setting new ones
        self.move_group.clear_path_constraints()
        target_pose.orientation = self.move_group.get_current_pose().pose.orientation
        
        # ## lift up the bottle and match the final orientation
        # lift_up_pose = self.move_group.get_current_pose().pose
        # lift_up_pose.orientation = target_pose.orientation
        # lift_up_pose.position.z = target_pose.position.z
        
        # # here we lift the cup a little firstly
        # self.move_to_pose(lift_up_pose)

                
        # Create orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = "virtual_ee_link"  # Make sure this matches your actual end-effector link name
        orientation_constraint.header.frame_id = "base_link"  # Make sure this matches your robot's base frame
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.2 # Allowable error on the X-axis
        orientation_constraint.absolute_y_axis_tolerance = 0.2 # Allowable error on the Y-axis
        orientation_constraint.absolute_z_axis_tolerance = 0.2 # Allow larger freedom on the Z-axis
        orientation_constraint.weight = 0.7

        # Add the orientation constraint to path constraints
        # constraints = Constraints()
        # constraints.orientation_constraints.append(orientation_constraint)
        # self.move_group.set_path_constraints(constraints)
        # self.move_group.set_planning_time(20)
        # self.move_group.set_planner_id("RRTconnect") 
        ## allow replanning
        self.move_group.allow_replanning(True)
        # Plan and execute the motion to the pouring position
        rospy.loginfo("Moving to pouring position with upward orientation constraint...")
        
        success = self.move_to_pose(target_pose)

        # Stop and clear constraints after execution
        self.move_group.clear_pose_targets()
        self.move_group.clear_path_constraints()

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
    
    def attach_bottle_to_hand(self, bottle_dimension):
        water_bottle = PoseStamped()
        water_bottle.header.frame_id = "virtual_ee_link"
        water_bottle.pose.orientation.x = 0
        water_bottle.pose.orientation.y = 0.70710678
        water_bottle.pose.orientation.z = 0
        water_bottle.pose.orientation.w = 0.70710678
        water_bottle.pose.position.x = -(0.02) # slightly above the end effector
        self.scene.add_cylinder( name = "attached_bottle", pose = water_bottle, height = bottle_dimension.z, radius = (bottle_dimension.x /2) )
        ## wait for the object to come in
        self.wait_for_state_update(box_name = "attached_bottle")
        touch_links =  robot.robot.get_link_names(group='endeffector')
        robot.scene.attach_box(link = "virtual_ee_link", name = "attached_bottle" ,touch_links=touch_links)
        self.wait_for_state_update(box_name = "attached_bottle")
    
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
        pick_pose.orientation.x = -0.71612        # go_orientation.orientation.x = -0.71612s
        pick_pose.orientation.y = 0.0175        # go_orientation.orientation.y = 0.0175
        pick_pose.orientation.z = 0.6976        # go_orientation.orientation.z = 0.6976
        pick_pose.orientation.w = -0.01        # go_orientation.orientation.w = -0.01

        print("The target pick_pose is:",pick_pose)

        return self.move_to_pose(pick_pose)
        

class Pose_Shape_Subscriber(object):
    def __init__(self):
        # Create storage for bottle and cup poses
        self.bottle_pose = Pose()
        self.cup_pose = Pose()

        # Flags to check if poses have been received
        self.bottle_pose_received = False
        self.cup_pose_received = False
        self.bottle_dimension_received = False
        self.cup_dimension_received = False        
        # Subscribe to the topics and assign callbacks
        rospy.Subscriber("transformed_cup_pose", PoseStamped, self.pose_cup_callback)
        rospy.Subscriber("transformed_bottle_pose", PoseStamped, self.pose_bottle_callback)
        # subsribe to the bottle and cup shape
        rospy.Subscriber("/dimension_cup", Vector3, self.dimension_cup_callback)
        rospy.Subscriber("/dimension_bottle", Vector3, self.dimension_bottle_callback)

    def pose_cup_callback(self, msg):
        """Callback to update the cup pose."""
        self.cup_pose = msg
        self.cup_pose_received = True
        rospy.loginfo("Updated cup pose.")
        
    def pose_bottle_callback(self, msg):
        """Callback to update the bottle pose."""
        self.bottle_pose = msg
        self.bottle_pose_received = True
        rospy.loginfo("Updated bottle pose.")
    
    def dimension_cup_callback(self, msg):
        "Callback to update the dimension of cup"
        self.cup_dimension = msg
        self.cup_dimension_received = True
    
    def dimension_bottle_callback(self, msg):
        "Callback to update the dimension of cup"
        self.bottle_dimension = msg
        self.bottle_dimension_received = True
        

    def get_bottle_pose_dimension(self):
        """Get the latest bottle pose if available, otherwise raise an exception."""
        if not self.bottle_pose_received or not self.bottle_dimension_received:
            rospy.logwarn("Bottle pose/dimension has not been received yet. Returning None.")
            return None, None
        rospy.loginfo("fetched bottle pose and dimension")
        return self.bottle_pose, self.bottle_dimension

    def get_cup_pose_dimension(self):
        """Get the latest cup pose if available, otherwise raise an exception."""
        if not self.cup_pose_received or not self.cup_dimension_received:
            rospy.logwarn("Cup pose/dimension has not been received yet. Returning None.")
            return None, None
        rospy.loginfo("fetched cup pose and dimension")
        return self.cup_pose, self.cup_dimension


if __name__ == '__main__':    
    try:
        ## one ros py file should have one and only one init_node
        rospy.init_node('robot_controller', anonymous=True)

        robot = RobotController()
        Pose_Shape_Subscriber = Pose_Shape_Subscriber()
        robot.gripper.open_gripper()
        robot.move_group.allow_replanning(True)

        ############################# use sequential stage to settle ##############################
        
        ###################### Stage 1 #####:::move to camera_position1 to detect object; camera_position1 is at right table
        robot.move_to_joint_goal([3.4034581184387207, -1.5696094793132325, 1.459726635609762, 1.2243940073200683, 1.642998218536377, -3.156583611165182])
        ## waite for the pose of cup come out
        rospy.sleep(5)
        while not rospy.is_shutdown():
            if Pose_Shape_Subscriber.cup_pose_received and Pose_Shape_Subscriber.cup_dimension_received:
                cup_pose, cup_dimension = Pose_Shape_Subscriber.get_cup_pose_dimension()
                break
            print("no cup pose recieved, waiting")
        ### some manual offset#####
        cup_pose.pose.position.x = cup_pose.pose.position.x - 0.05
        #cup_pose.pose.position.z = cup_pose.pose.position.z - 0.03

        print("target cup pose is:",cup_pose)
        print("target cup dimension is:",cup_dimension)
        
        # robot.move_to_object_pose(cup_pose.pose)
        # sys.exit()

        ####################### Stage 2 ####:::move to camera_position2 to detect object
        robot.move_to_joint_goal([2.7063164710998535, -1.5696094793132325, 1.4597743193255823, 1.2243701654621582, 1.6429743766784668, -3.156583611165182])
        ## waite for the pose of bottle come out
        rospy.sleep(5)
        while not rospy.is_shutdown():
            if Pose_Shape_Subscriber.bottle_pose_received and Pose_Shape_Subscriber.bottle_dimension_received:
                bottle_pose, bottle_dimension = Pose_Shape_Subscriber.get_bottle_pose_dimension()
                break
            print("no bottle pose recieved, waiting")
        
        ### some manual offset######
        ##bottle_pose.pose.position.y = bottle_pose.pose.position.y - 0.03
    
        
        print("target bottle pose is:",bottle_pose)
        print("target bottle dimension is:",bottle_dimension)

        ########################## Stage 3 #### :::: TODO: a pregrasping position, avoid collision
        ## we should firstly generate the bottle in the scene, to avoid collision for pre-grasping position
        initial_bottle_pose = copy.deepcopy(bottle_pose)
        initial_bottle_pose.pose.orientation.x = 0
        initial_bottle_pose.pose.orientation.y = 0
        initial_bottle_pose.pose.orientation.z = 0
        initial_bottle_pose.pose.orientation.w = 1
        robot.scene.add_cylinder(name =  "initial_bottle", pose = initial_bottle_pose, height = bottle_dimension.z + 0.03, radius = (bottle_dimension.x /2) )
        ## wait for the object to come in
        robot.wait_for_state_update(box_name = "initial_bottle")
        rospy.sleep(2)
        #deep copy the pregrasping pose
        pre_grasping_pose = copy.deepcopy(bottle_pose)
        pre_grasping_pose.pose.position.x = pre_grasping_pose.pose.position.x + 0.05
        success_1 = robot.move_to_object_pose(pre_grasping_pose.pose)
        #then, remove the cup from the world scene
        rospy.sleep(5)
        
        robot.scene.remove_world_object("initial_bottle")
        robot.wait_for_state_update("initial_bottle")
        
        ########################### Stage 4 ###### ::: grasp bottle, attach the bottle to the hand to avoid collision
        ### TODO: go to grasp bottle
        grasping_pose = copy.deepcopy(bottle_pose)
        grasping_pose.pose.position.z = grasping_pose.pose.position.z
        robot.move_to_object_pose(grasping_pose.pose)
        robot.gripper.close_gripper()
        
        ## after grasping, lift up a little to avoid collide with table
        lift_up = robot.move_group.get_current_pose().pose
        lift_up.position.z = lift_up.position.z + 0.1
        robot.move_to_pose(lift_up)
        ### re-initialize the bottle and attach it to the robot hand
        robot.attach_bottle_to_hand(bottle_dimension= bottle_dimension)
        
        ######################## Stage 5 ######## :::: TODO: cacluclate and move to the optimisatized-offset position
        #### from now on, use the tip of bottle as EE
        tip_pose = robot.cal_tip_from_ee(ee_pose = robot.move_group.get_current_pose().pose)
        start_pose = copy.deepcopy(tip_pose) 
        target_pose = copy.deepcopy(cup_pose)
        ## here calculate the target_pose upward the cup
        target_pose.pose.position.z =  target_pose.pose.position.z + cup_dimension.z / 2 + bottle_dimension.z / 2

        print("the upward cup target_pose is:",target_pose)
        ## the offset, circle radius:
        offset_radius = cup_dimension.y/2 + bottle_dimension.y/2 + 0.05
        print("the offset radius is:",offset_radius)
        
        ############## invoke the optimizer #########
        theta, distance, optimized_offset_pose = offset_pose_optimiser(target_pose.pose, start_pose, offset=offset_radius)
        print("the optimized result is:", theta, distance, optimized_offset_pose)
        ## calculate back to robot ee pose
        optimized_offset_pose_ee = robot.cal_ee_from_tip(optimized_offset_pose)

        #robot.move_to_pose(optimized_offset_pose)
        ## cheat here
        success = robot.move_to_object_pose(optimized_offset_pose_ee)
        if success is False:
            sys.exit()
            
        robot.scene.remove_attached_object("virtual_ee_link", name="attached_bottle")
        robot.scene.remove_world_object("attached_bottle")
        sys.exit()    
        
        
        #robot.move_group.set_planner_id("RRTconnect") 
            
        ############# Stage 6 ##########::: Pouring action, input: \theta, the target_pose
        ## monitor the initial weight
        # if robot.initial_weight is None:
        #     robot.initial_weight = robot.current_weight
        #     print("self.initial_weight is", robot.initial_weight)
        # ## cartrsian path waypoints            
        waypoints = []
        
        wpose = robot.move_group.get_current_pose().pose
        tip_position = robot.cal_tip_from_ee(copy.deepcopy(wpose))
        
        tip_position.position.x =  target_pose.pose.position.x
        tip_position.position.y =  target_pose.pose.position.y
        tip_position.position.z =  target_pose.pose.position.z
        
        tip_position_rotate = robot.rotation_plus_euler(current_pose= copy.deepcopy(tip_position), euler_added= [-theta,0,-1.5])
        wpose = robot.cal_ee_from_tip(copy.deepcopy(tip_position_rotate))
        
        waypoints.append(copy.deepcopy(wpose))
        
        tip_position_2 = robot.cal_tip_from_ee(copy.deepcopy(wpose))
        
        tip_position_2.position.x =  target_pose.pose.position.x
        tip_position_2.position.y =  target_pose.pose.position.y
        tip_position_2.position.z =  target_pose.pose.position.z - 0.08
        
        tip_position_rotate_2 = robot.rotation_plus_euler(current_pose= copy.deepcopy(tip_position_2), euler_added= [0,0,-1])
        wpose_2 = robot.cal_ee_from_tip(copy.deepcopy(tip_position_rotate_2))

        waypoints.append(copy.deepcopy(wpose_2))
        
        (plan, fraction) = robot.move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.0001,        # eef_step
                                        0)         # jump_threshold
        print("the cartesian path fraction is", fraction)
        
        ## !! we set the execture to be asynchonize that allow to run the following program  while runing .move_group.execute the last plan
        robot.move_group.execute(plan, wait=True) 
        
        # robot.monitor_weight_loss()
        
        rospy.loginfo("Trajectory completed or stopped due to weight loss.")
        
        robot.scene.remove_attached_object("virtual_ee_link", name="attached_bottle")
        robot.scene.remove_world_object("attached_bottle")
        rospy.sleep(2)
        
        robot.move_to_joint_goal([3.28159016, -1.54217293, 2.24431889, -0.679631211, 1.65073241, -3.13583307])

    except rospy.ROSInterruptException:
        robot.scene.remove_attached_object("virtual_ee_link", name="attached_bottle")
        robot.scene.remove_world_object("attached_bottle")
        rospy.sleep(2)

        pass
