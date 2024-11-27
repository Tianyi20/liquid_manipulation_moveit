#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from gripper_control import Gripper
import sys  
from moveit_msgs.msg import OrientationConstraint, Constraints

class RobotController(object):
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        # Initialize the move group for the manipulator
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        group_names = self.robot.get_group_names()
        print("Available groups:", group_names)
        
        self.move_group.set_planner_id("PRMkConfigDefault")
        
        current_planner_id = self.move_group.get_planner_id()
        print("Current planner ID:", current_planner_id)
        
        #self.move_group.setPlanningPipelineId("RRTstarkConfigDefault")
        
        # This will store the pose received from the subscriber
        self.object_pose = None
        ## import the gripper class
        self.gripper = Gripper()
        self.box_name = "box"

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
        
        # 2-> Move to middle position
        self.move_to_parrallel_target(self.object_pose)
        # Once the pose is received, move to the object's position
        rospy.loginfo("Successfully reached the middle state of the object target")
        rospy.loginfo("ready to go down")
        rospy.sleep(2)
        
        # 3-> Go down to target position
        self.move_to_object_pose(self.object_pose)
        
        rospy.loginfo("Successfully reached the object target")
        
        self.gripper.close_gripper()
        rospy.loginfo("close gripper!")


        #todo: move to the fixed pouring position with constrained preventing water leakage

        #todo: only change the joint_6 to make a good pouring action

        #todo: change back joint_6 to be upward

        #use forward kinematics to place the soup on table again

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
        pick_pose.position.z = object_pose.position.z + 0.03
        
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
    
    def move_to_parrallel_target(self, object_pose):
        """
        Move the robot to position that X,Y same to target, but z as the robot current pose
        """        
        current_pose = self.move_group.get_current_pose().pose
        middle_use_pose = Pose()
        middle_use_pose.position.x = object_pose.position.x
        middle_use_pose.position.y = object_pose.position.y
        middle_use_pose.position.z = current_pose.position.z - 0.1
        ##This orientation can keep the tool be upward and pointing outside the table
        middle_use_pose.orientation.x = -0.71612
        middle_use_pose.orientation.y = 0.0175
        middle_use_pose.orientation.z = 0.6976
        middle_use_pose.orientation.w = -0.01
        print("The target middle use between the pick_pose is:",middle_use_pose)

        return self.move_to_pose(middle_use_pose)
        
    def move_to_camera_position(self):
        """
        Move to a predefined camera scanning position using forward kinematics.
        """
        joint_goal = [3.14, -1.57, 1.66, 1.05, 1.62, -3.07]  # Example joint goal for camera position
        return self.move_to_joint_goal(joint_goal)
    
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
        orientation_constraint.weight = 0.7

        # Add the orientation constraint to path constraints
        constraints = Constraints()
        constraints.orientation_constraints.append(orientation_constraint)
        self.move_group.set_path_constraints(constraints)
        self.move_group.set_planner_id("RRTstarkConfigDefault") 
        # Increase planning time for a better chance of finding a solution
        self.move_group.set_planning_time(20)
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
    
    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene at the location of the left finger:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "virtual_ee_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0 # slightly above the end effector
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.1, 0.06, 0.06))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = 'endeffector'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box("virtual_ee_link", box_name, touch_links=touch_links)
        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


if __name__ == '__main__':
    try:
        robot = RobotController()

        # Step 1: Move to the camera scan position using joint goals (forward kinematics)
        success_stage1 = robot.move_to_camera_position()
        
        if not success_stage1:
            rospy.logwarn("Failed to reach the camera scan position. Exiting program.")
            sys.exit(1)  

        rospy.sleep(3)
        
        go_orientation = Pose()
        go_orientation.position.x = -0.571212573369
        go_orientation.position.y = 0.2
        go_orientation.position.z = 0.1
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

        # self.grasp_pose.orientation.x = -0.719093153708
        # self.grasp_pose.orientation.y = -0.694812369689
        # self.grasp_pose.orientation.z = 0.00206665742698
        # self.grasp_pose.orientation.w = 0.0116848682052
        robot.move_to_pose(go_orientation)

        robot.add_box()
        success = robot.attach_box()
        if success:
            print("succesfully attach the soup")
        else:
            print("Fail to attach tomatto soup")
        success_stage_pouring = robot.move_to_pouring_position()
        
    except rospy.ROSInterruptException:
        pass
