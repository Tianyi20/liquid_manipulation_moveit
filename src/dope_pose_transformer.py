#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix, translation_matrix, translation_from_matrix

# Global publisher
transformed_pose_pub = None

def pose_cup_callback(msg):
    if msg is not None:
        # Initialize the TF buffer
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        print("I hear the pose_cup")

        try:
            # Get the transform from 'realsense_camera_link' to 'base_link'
            transform_stamped = tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(1))

            # Extract translation and rotation from the transform
            translation = transform_stamped.transform.translation
            rotation = transform_stamped.transform.rotation

            # Create transformation matrices from translation and rotation
            translation_mat = translation_matrix([translation.x, translation.y, translation.z])
            rotation_mat = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

            # Combine translation and rotation into a transformation matrix
            transform_mat = translation_mat.dot(rotation_mat)

            # Extract pose from the incoming message
            pose_translation = translation_matrix([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            pose_rotation = quaternion_matrix([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

            # Combine the translation and rotation of the pose
            pose_mat = pose_translation.dot(pose_rotation)

            # Apply the transformation to the pose
            transformed_pose_mat = transform_mat.dot(pose_mat)

            # Extract the transformed translation and rotation
            transformed_translation = translation_from_matrix(transformed_pose_mat)
            transformed_rotation = quaternion_from_matrix(transformed_pose_mat)

            # Construct a new PoseStamped message with the transformed data
            transformed_pose = PoseStamped()
            transformed_pose.header.stamp = rospy.Time.now()
            transformed_pose.header.frame_id = 'base_link'
            transformed_pose.pose.position.x = transformed_translation[0]
            transformed_pose.pose.position.y = transformed_translation[1]
            transformed_pose.pose.position.z = transformed_translation[2]
            transformed_pose.pose.orientation.x = transformed_rotation[0]
            transformed_pose.pose.orientation.y = transformed_rotation[1]
            transformed_pose.pose.orientation.z = transformed_rotation[2]
            transformed_pose.pose.orientation.w = transformed_rotation[3]

            # Publish the transformed pose
            transformed_cup_pub.publish(transformed_pose)

            # Log the transformed pose for debugging
            rospy.loginfo("Transformed [Cup] Pose in base_link: %s", transformed_pose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF transform lookup failed: %s" % str(e))
        
        
def pose_bottle_callback(msg):
    if msg is not None:
        # Initialize the TF buffer
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        print("I hear the pose_bottle")

        try:
            # Get the transform from 'realsense_camera_link' to 'base_link'
            transform_stamped = tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(1))

            # Extract translation and rotation from the transform
            translation = transform_stamped.transform.translation
            rotation = transform_stamped.transform.rotation

            # Create transformation matrices from translation and rotation
            translation_mat = translation_matrix([translation.x, translation.y, translation.z])
            rotation_mat = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

            # Combine translation and rotation into a transformation matrix
            transform_mat = translation_mat.dot(rotation_mat)

            # Extract pose from the incoming message
            pose_translation = translation_matrix([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            pose_rotation = quaternion_matrix([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

            # Combine the translation and rotation of the pose
            pose_mat = pose_translation.dot(pose_rotation)

            # Apply the transformation to the pose
            transformed_pose_mat = transform_mat.dot(pose_mat)

            # Extract the transformed translation and rotation
            transformed_translation = translation_from_matrix(transformed_pose_mat)
            transformed_rotation = quaternion_from_matrix(transformed_pose_mat)

            # Construct a new PoseStamped message with the transformed data
            transformed_pose = PoseStamped()
            transformed_pose.header.stamp = rospy.Time.now()
            transformed_pose.header.frame_id = 'base_link'
            transformed_pose.pose.position.x = transformed_translation[0]
            transformed_pose.pose.position.y = transformed_translation[1]
            transformed_pose.pose.position.z = transformed_translation[2]
            transformed_pose.pose.orientation.x = transformed_rotation[0]
            transformed_pose.pose.orientation.y = transformed_rotation[1]
            transformed_pose.pose.orientation.z = transformed_rotation[2]
            transformed_pose.pose.orientation.w = transformed_rotation[3]

            # Publish the transformed pose
            transformed_bottle_pub.publish(transformed_pose)

            # Log the transformed pose for debugging
            rospy.loginfo("Transformed [Bottle] Pose in base_link: %s", transformed_pose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF transform lookup failed: %s" % str(e))

def main():
    global transformed_cup_pub
    
    global transformed_bottle_pub

    rospy.init_node('dope_pose_transformer')

    # Create a publisher for the transformed pose
    transformed_cup_pub = rospy.Publisher('transformed_cup_pose', PoseStamped, queue_size=1)
    
    transformed_bottle_pub = rospy.Publisher('transformed_bottle_pose', PoseStamped, queue_size=1)

    # Subscribe to the 'dope/pose_soup' topic (PoseStamped message type)
    rospy.Subscriber("/pose_cup", PoseStamped, pose_cup_callback)

    rospy.Subscriber("/pose_bottle", PoseStamped, pose_bottle_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
