#!/usr/bin/env python 
import rospy
import tf
import tf.transformations
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import yaml
import rospkg
import os

class InitialPoseEstimate:
    def __init__(self):
        rospy.init_node("initial_pose_publisher")
        rospack = rospkg.RosPack()
        try:
            pack_dir = rospack.get_path("broadcast_tf")
            self.config_file = os.path.join(pack_dir, "config/tag_pose.yaml")

        except:
            self.config_file = ""
            rospy.logwarn_once("Error finding required package")

        self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.pub = 0
        self.current_tag = ""
        self.tf_ros = tf.TransformerROS()

    def tf_matrix(self, trans, rot):
        return self.tf_ros.fromTranslationRotation(trans, rot)
    
    def get_tag_poses(self):
        # Path to the config file
        if self.config_file != "": 
            with open(self.config_file, "r") as f:
                data = yaml.safe_load(f)
            return data
        
    def tf_cam_to_base(self):
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                trans, rot = listener.lookupTransform("camera_rgb_optical_frame", "base_link", rospy.Time(0))
                trans = (np.round(trans, 2)).tolist()
                rot = (np.round(rot, 2)).tolist()
                res = self.tf_matrix(trans, rot)
                return res
        
            except:
                pass
        
    def callback(self, data):
        initial_pose = PoseWithCovarianceStamped()

        # Read tag pose in YAML file
        tag_poses = self.get_tag_poses()
        
        # Calculate cam to tag pose
        for detection in data.detections:
            tag_id = detection.id[0]
            tag_frame = "tag_" + str(tag_id)
            x = detection.pose.pose.pose.position.x
            y = detection.pose.pose.pose.position.y
            z = detection.pose.pose.pose.position.z
            qx = detection.pose.pose.pose.orientation.x
            qy = detection.pose.pose.pose.orientation.y
            qz = detection.pose.pose.pose.orientation.z
            qw = detection.pose.pose.pose.orientation.w
            trans = [x,y,z]
            rot = [qx, qy, qz, qw]
            tf_cam_to_tag = np.linalg.inv(self.tf_matrix(trans, rot))

            # Get Tag poses from YAML file
            if tag_frame in tag_poses.keys():
                t_x = tag_poses[tag_frame]["x"]
                t_y = tag_poses[tag_frame]["y"]
                t_z = tag_poses[tag_frame]["z"]
                r_x = tag_poses[tag_frame]["qx"]
                r_y = tag_poses[tag_frame]["qy"]
                r_z = tag_poses[tag_frame]["qz"]
                r_w = tag_poses[tag_frame]["qw"]
                tag_trans = [t_x, t_y, t_z]
                tag_rot = [r_x, r_y, r_z, r_w]
                tf_tag_to_map = self.tf_matrix(tag_trans, tag_rot)
            try:
                # Calculate Camera Pose
                cam_pose = np.matmul(tf_tag_to_map, tf_cam_to_tag)
                tf_tmp = self.tf_cam_to_base()

                # Calculate robot pose
                pose = np.matmul(cam_pose, tf_tmp)
                pose = np.round(pose, 2)

                rot_euler = tf.transformations.euler_from_matrix(pose[0:3,0:3])
                rot_q_map = tf.transformations.quaternion_from_euler(0,0, rot_euler[2])
               
                
                # Declare message to publish initial pose
                initial_pose.header.frame_id = "map"
                initial_pose.pose.pose.position.x = pose[0,3]
                initial_pose.pose.pose.position.y = pose[1,3]
                initial_pose.pose.pose.orientation.w = rot_q_map[3]
                initial_pose.pose.pose.orientation.z = rot_q_map[2]
                # Set the covariance is optional (in this case, we use AMCL as the main localization so covariance is still set for the AMCL to converge)
                initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]


                # Publish Initial Pose
                if self.pub < 10: 
                    self.pose_pub.publish(initial_pose)
                    self.pub += 1
                    self.current_tag = tag_frame
                else: 
                    rospy.loginfo_once("Initial Pose Published Successfully")
                    if self.current_tag != tag_frame:
                        self.pub = 0
            except:
                print("Error")
                pass


if __name__=="__main__":
    try: 
        tag_record = InitialPoseEstimate()
        rospy.spin()
    except:
        print("Error")
