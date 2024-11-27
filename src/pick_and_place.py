#!/usr/bin/env python

import rospy
import rospkg
import numpy as np

from control_wrapper.srv import SetPose
from control_wrapper.srv import GetPose
from control_wrapper.srv import SetJoints
from control_wrapper.srv import SetTrajectory
from collaborative_cdp.srv import PickPlace, PickPlaceRequest, PickPlaceResponse

from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
# from tri_star.msg import TargetPositions
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
# from tri_star.robot_util import Robot, pose_matrix_to_msg
from std_msgs.msg import String, Bool
from gripper_control import Gripper

SLEEP_BETWEEN_MOVEMENTS = 0.5

# class Gripper(object):
#     def __init__(self):
#         self.topic = "/" + "ur" + "/control_wrapper/" + "left" + "/"
#         print("Waiting for service...")
#         rospy.wait_for_service("/ur/control_wrapper/left/gripper_srv")
#         self.gripper_sp = rospy.ServiceProxy("/ur/control_wrapper/left/gripper_srv", SetBool)
#         print("Service started!")

#     def open_gripper(self):
#         self.rsp = self.gripper_sp(True)
#         print(self.rsp.success)
#         print("Publsihing gripper open!")

#     def close_gripper(self):
#         self.rsp = self.gripper_sp(False)
#         print(self.rsp.success)
#         print("Publsihing gripper close!")

class PickAndPlace(object):
    def __init__(self):

        self._put_down = rospy.get_param("chefbot/manipulation/put_down", default=False)
        self._down_depth = rospy.get_param("chefbot/manipulation/down_depth", default=0.1)

        self.connect_pub = rospy.Publisher("/ur/control_wrapper/left/connect", Bool, queue_size=10)

        # Robot IK service proxies
        rospy.wait_for_service("/ur/control_wrapper/left/get_pose")
        self.get_robot_pose_sp = rospy.ServiceProxy("/ur/control_wrapper/left/get_pose", GetPose)

        rospy.wait_for_service("/ur/control_wrapper/left/set_pose")
        self.set_robot_pose_sp = rospy.ServiceProxy("/ur/control_wrapper/left/set_pose", SetPose)

        # Pick and Place service
        self.pick_place_srv = rospy.Service("/chefbot/manipulation/pick_place", PickPlace, self.pick_place_srv_cb)

        # Obtain the current robot pose
        self.current_pose = self.get_robot_pose_sp()

        # Open gripper initially
        self.gripper = Gripper()
        # self.gripper.open_gripper()

        self.grasp_pose = Pose()
        self.grasp_pose.orientation.x = -0.719093153708
        self.grasp_pose.orientation.y = -0.694812369689
        self.grasp_pose.orientation.z = 0.00206665742698
        self.grasp_pose.orientation.w = 0.0116848682052

        self.xy_pick_pose = None
        self.z_pick_down_pose = None
        self.z_pick_up_pose = None

        self._dist_thresh = .01 # cm

    def _check_dest_is_reached(self, dest):
        cur_pos = self.get_robot_pose_sp()
        cur_pos = np.array([cur_pos.pose.position.x, cur_pos.pose.position.y,
                            cur_pos.pose.position.z])
        dest = np.array([dest.position.x, dest.position.y, dest.position.z])

        dist = np.linalg.norm(dest - cur_pos)

        return dist <= self._dist_thresh

    def _move_to_xy_obj_pos(self):
        self.gripper.open_gripper()
        print("Gripper Opened!")
        self.xy_pick_pose = self.assign_pose(self.object_pose.position.x,
                                        self.object_pose.position.y,
                                        self.start_pose.position.z,
                                        self.grasp_pose.orientation)
        reached_xy_pick = self.set_robot_pose_sp(request_pose=self.xy_pick_pose)
        # return reached_xy_pick

        return self._check_dest_is_reached(self.xy_pick_pose)

    def _move_down_to_grasp_obj(self):
        self.z_pick_down_pose = self.assign_pose(self.xy_pick_pose.position.x,
                                            self.xy_pick_pose.position.y,
                                            self.object_depth,
                                            self.grasp_pose.orientation)
        reached_z_pick_down = self.set_robot_pose_sp(request_pose=self.z_pick_down_pose)

        # return reached_z_pick_down
        return self._check_dest_is_reached(self.z_pick_down_pose)

    def _lift_obj_to_height(self):
        self.gripper.close_gripper()
        print("Gripper Closed!")
        self.connect_pub.publish(True)
        self.z_pick_up_pose = self.assign_pose(self.xy_pick_pose.position.x,
                                            self.xy_pick_pose.position.y,
                                            self.destination_pose.position.z,
                                            self.grasp_pose.orientation)

        reached_z_pick_up = self.set_robot_pose_sp(request_pose=self.z_pick_up_pose)
        # return reached_z_pick_up
        return self._check_dest_is_reached(self.z_pick_up_pose)

    def _move_to_dest_xy(self):
        self.xy_place_up_pose = self.assign_pose(self.destination_pose.position.x,
                                            self.destination_pose.position.y,
                                            self.destination_pose.position.z,
                                            self.grasp_pose.orientation)

        reached_xy_place_up = self.set_robot_pose_sp(request_pose=self.xy_place_up_pose)
        # return reached_xy_place_up

        return self._check_dest_is_reached(self.xy_place_up_pose)

    def _place_obj_down(self):
        self.z_place_down_pose = self.assign_pose(self.destination_pose.position.x,
                                            self.destination_pose.position.y,
                                            self.down_depth,
                                            self.grasp_pose.orientation)
        reached_z_place_down = self.set_robot_pose_sp(request_pose=self.z_place_down_pose)

        if reached_z_place_down.is_reached:
            self.gripper.open_gripper()
            self.connect_pub.publish(True)

        # return reached_z_place_down
        return self._check_dest_is_reached(self.z_place_down_pose)

    def pick_place_srv_cb(self, data):
        self.start_pose = data.start_pose
        self.object_pose = data.object_pose
        self.destination_pose = data.destination_pose
        self.object_depth = data.object_depth
        self.down_depth = self._down_depth

        self.pick_success = False
        self.place_success = False
        import pdb; pdb.set_trace()
        step_1_success = self._move_to_xy_obj_pos()
        print("Step_1_success = {}".format(step_1_success))
        # If step 1 fails, we want to action interface to reattempt
        if not step_1_success:
            return PickPlaceResponse(picked_object=False,
                                     placed_object=False)

        step_2_success = self._move_down_to_grasp_obj()
        print("Step_2_success = {}".format(step_2_success))

        step_3_success = self._lift_obj_to_height()
        while not step_3_success:
            print("Step_4 failed, reattemptting...")
            step_3_success = self._move_to_dest_xy()
        print("Step_3_success = {}".format(step_3_success))

        self.pick_success = (step_1_success and
                             step_2_success and
                             step_3_success)
        rospy.sleep(1)
        step_4_success = self._move_to_dest_xy()
        print("Step_4_success = {}".format(step_4_success))
        while not step_4_success:
            print("Step_4 failed, reattemptting...")
            step_4_success = self._move_to_dest_xy()

        if self._put_down:
            step_5_success = self._place_obj_down()
        else:
            step_5_success = True
        print("Step_5_success = {}".format(step_5_success))

        self.place_success = (step_4_success and
                              step_5_success)
        
        self.gripper.open_gripper()
        print("Gripper Opened!")

        return PickPlaceResponse(picked_object=self.pick_success,
                                 placed_object=self.place_success)

    def assign_pose(self, x, y, z, orientation):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation = orientation
        return pose

    def run(self):
        pass

if __name__ == '__main__':
    try:
            rospy.init_node('pick_and_place', anonymous=True)

            pick_place = PickAndPlace()
            while not rospy.is_shutdown():
                pick_place.run()


    except rospy.ROSInterruptException:
        pass
