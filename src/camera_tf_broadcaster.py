#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped

def publish_camera_tf():
    rospy.init_node('camera_tf_broadcaster')
    
    # TransformBroadcaster
    br = tf2_ros.TransformBroadcaster()
    
    # 
    t = TransformStamped()

    # 
    t.header.frame_id = "virtual_ee_link"
    # 
    t.child_frame_id = "camera_color_optical_frame"
    
    rate = rospy.Rate(1.0)  # 1000Hz
    while not rospy.is_shutdown():
        # 
        t.header.stamp = rospy.Time.now()

        # 
        # t.transform.translation.x = -0.03
        # t.transform.translation.y = -0.0325
        # t.transform.translation.z = -0.07

        # # [0, 0, 0, 1])
        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = 0.0
        # t.transform.rotation.w = 1.0
        t.transform.translation.x = -0.03
        t.transform.translation.y = 0.0325
        t.transform.translation.z = -0.07

        # [0, 0, 0, 1])
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = -0.70710678
        t.transform.rotation.w = 0.70710678
        # 
        br.sendTransform(t)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_tf()
    except rospy.ROSInterruptException:
        pass
