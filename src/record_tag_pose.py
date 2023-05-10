#!/usr/bin/env python 
import rospy
import tf
import tf.transformations
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
import yaml
import rospkg
import os

class TagPoseRecorder:
    def __init__(self):
        rospy.init_node("tag_pose_recorder")
        rospack = rospkg.RosPack()

        try:
            pack_dir = rospack.get_path("broadcast_tf")
            self.config_file = os.path.join(pack_dir, "config/tag_pose.yaml")

            if not os.path.exists(self.config_file):
                with open(self.config_file, "w"):
                    pass

        except:
            self.config_file = ""
            print("Error finding required package")
        self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)
        self.listener = tf.TransformListener()
        

    def callback(self, data):
        tag_poses = {}
        for detection in data.detections: 
            tag_id = detection.id[0]
            tag_frame = "tag_" + str(tag_id)
            try:
                trans, rot = self.listener.lookupTransform("map", tag_frame, rospy.Time(0))
                trans = (np.round(trans, 2)).tolist()
                rot = (np.round(rot, 2)).tolist()

                print('Translation:', trans)
                print('Rotation:', rot)
                print("Rotation in RPY: ", np.round(tf.transformations.euler_from_quaternion(rot),2))

                pose_dict = {
                    "x": trans[0],
                    "y": trans[1],
                    "z": trans[2],
                    "qx": rot[0],
                    "qy": rot[1],
                    "qz": rot[2],
                    "qw": rot[3],
                    "frame_id": tag_frame
                    }
                tag_poses[tag_frame] = pose_dict
            except tf.Exception:
                rospy.logwarn_once("No map found")

        # Writing tag pose to YAML file (config file located in the broadcast_tf package)
        if self.config_file != "":
            with open(self.config_file, "r+") as f:
                data = yaml.safe_load(f)
                if data != None:
                    tags_write = data.keys()
                    tags_seen = tag_poses.keys()
                    for tag in tags_seen:
                        if tag not in tags_write:
                            yaml.dump({tag: tag_poses[tag]}, f)
                elif tag_poses != {}:
                    yaml.dump(tag_poses, f)
        else: 
            rospy.logwarn_once("No config file found")
                
            
if __name__=="__main__":
    try: 
        tag_record = TagPoseRecorder()
        rospy.spin()
    except:
        print("Error")
