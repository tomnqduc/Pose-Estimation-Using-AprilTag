#!/usr/bin/env python
import rospy
import tf
import tf.transformations
import sys
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np

def main(target, source):

    rospy.init_node("tf_listener")
    pose_sub = rospy.Subscriber("/tag_detectiokns_throttle", AprilTagDetectionArray, callback(target, source))
    rospy.spin()

def callback(target,source):
    print(find_transform(target, source))
    print("Hello")
def find_transform(target, source):
    listener = tf.TransformListener()
    # rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
    # while listener.lookupTransform(target, source, rospy.Time(0)) == None:
        try:
            # Look up the transform
            (trans, rot) = listener.lookupTransform(target, source, rospy.Time(0)) #target, source
            # Print the transform
            print('Translation:', np.round(trans,2))
            print('Rotation:', np.round(rot,2))
            print("Rotation in RPY: ", np.round(tf.transformations.euler_from_quaternion(rot),2))
            return (trans, rot)
        except (tf.Exception):
            rospy.loginfo("Error lookup frame")

    # Sleep for the remaining time to achieve the desired loop rate
    # rate.sleep()



if __name__ =="__main__":
    main(sys.argv[1], sys.argv[2])
    