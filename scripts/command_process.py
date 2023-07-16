#!/usr/bin/env python 

import rospy
import tf
import tf.transformations
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import yaml
import sys
import rospkg
import os

class SetGoal():
    def __init__(self):
        rospy.init_node("goal_publisher_from_command")
        rospack = rospkg.RosPack()
        try:
            pack_dir = rospack.get_path("broadcast_tf")
            self.config_file = os.path.join(pack_dir, "config/tag_pose.yaml")

        except:
            self.config_file = ""
            rospy.logwarn_once("Error finding required package")

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.command_sub = rospy.Subscriber("/command", String, self.goal_publisher)
        self.tf_ros = tf.TransformerROS()
        self.count = 0

    def tf_matrix(self, trans, rot):
        return self.tf_ros.fromTranslationRotation(trans, rot)
    
    def get_tag_poses(self):
        # Path to the config file
        if self.config_file != "": 
            with open(self.config_file, "r") as f:
                data = yaml.safe_load(f)
            return data

    def goal_publisher(self, msg):
        if msg.data == "LABEL_1":
            tag_id = "tag_0"
        elif msg.data == "LABEL_4":
            tag_id = "tag_5"
        elif msg.data == "LABEL_3":
            tag_id = "tag_15"

        print(msg.data)
        tag_pose = self.get_tag_poses()

        trans = [0, 0, 0.75]
        rot = [0, 0, 0, 1]
        tf_goal_2_tag = self.tf_matrix(trans, rot)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"

        try: 
            if tag_id not in tag_pose.keys():
                rospy.logwarn_once("No tag found in the map")
            elif tag_id in tag_pose.keys():
                t_x = tag_pose[tag_id]["x"]
                t_y = tag_pose[tag_id]["y"]
                t_z = tag_pose[tag_id]["z"]
                r_x = tag_pose[tag_id]["qx"]
                r_y = tag_pose[tag_id]["qy"]
                r_z = tag_pose[tag_id]["qz"]
                r_w = tag_pose[tag_id]["qw"]
                tag_trans = [t_x, t_y, t_z]
                tag_rot = [r_x, r_y, r_z, r_w]
                tf_tag_to_map = self.tf_matrix(tag_trans, tag_rot)

                goal = np.matmul(tf_tag_to_map, tf_goal_2_tag)
                goal_orientation = tf.transformations.euler_from_matrix(goal[0:3,0:3])
                goal_orientation = tf.transformations.quaternion_from_euler(0, 0, goal_orientation[2])

                goal_pose.pose.position.x = goal[0,3]
                goal_pose.pose.position.y = goal[1,3]
                goal_pose.pose.orientation.z = goal_orientation[2]
                goal_pose.pose.orientation.w = goal_orientation[3]

                # print(goal_orientation)
                # print(goal_pose)
            
                self.goal_pub.publish(goal_pose)
        except:
            print("Do nothing")

if __name__ == "__main__":
    i = 0
    goal_pub = SetGoal()
    rospy.spin()