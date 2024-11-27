#!/usr/bin/env python

import rospy
import rospkg
import numpy as np

from control_wrapper.srv import SetPose
from control_wrapper.srv import GetPose
from control_wrapper.srv import SetJoints
from control_wrapper.srv import SetTrajectory

from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from std_msgs.msg import String, Bool


class Gripper(object):
    def __init__(self):
        "docstring"
        self.topic = "/" + "ur" + "/control_wrapper/" + "left" + "/"
        print("Waiting for service...")
        rospy.wait_for_service("/ur/control_wrapper/left/gripper_srv")
        self.gripper_sp = rospy.ServiceProxy("/ur/control_wrapper/left/gripper_srv", SetBool)
        print("[Gripper Control] Gripper Service started!")

    def open_gripper(self):
        # self.gripper_pub.publish(True)
        self.rsp = self.gripper_sp(True)
        print(self.rsp.success)
        print("Publsihing gripper open!")

    def close_gripper(self):
        self.rsp = self.gripper_sp(False)
        print(self.rsp.success)
        print("Publsihing gripper close!")