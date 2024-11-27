#!/usr/bin/env python

import tf
from geometry_msgs.msg import PoseStamped, Pose, Quaternion


def rotation_plus_euler(euler_added):
    """
    maintain the current pose, but only change the euler angle
    """
    current_pose = Pose()        
    current_pose.orientation.x = -0.71612
    current_pose.orientation.y = 0.0175
    current_pose.orientation.z = 0.6976
    current_pose.orientation.w = -0.01
    target_pose = Pose()
    target_pose.position = current_pose.position

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
 
    target_pose.orientation.x = target_orientation_q[0]
    target_pose.orientation.y = target_orientation_q[1]
    target_pose.orientation.z = target_orientation_q[2]
    target_pose.orientation.w = target_orientation_q[3]

    # move to new pose
    return target_pose

if __name__ == '__main__':

    print(rotation_plus_euler([1,0,0]))