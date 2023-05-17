#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/package.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <tf/tf.h>

// Declare global variable for the program
bool published = false;
std::string current_tag = "";
ros::Publisher pose_pub;

// Function to convert translation and quaternion rotation to homogenous tranformation matrix
Eigen::Matrix4f transformation_matrix(Eigen::Vector3f trans, Eigen::Quaternionf rot){

    Eigen::Matrix3f R = rot.toRotationMatrix();
    Eigen::Matrix4f transformation;
    Eigen::Vector4f tmp(0,0,0,1);

    transformation.block(0, 3, 3, 1) = trans;
    transformation.block(0, 0, 3, 3) = R;
    transformation.block(3, 0, 1, 4) = tmp.transpose();

    return transformation;
}

// Function return the transformation between camera and base link
Eigen::Matrix4f tf_cam_to_base() {
    // Create a transform listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Define the source and target frame names
    std::string sourceFrame = "base_link";
    std::string targetFrame = "camera_rgb_optical_frame";

    // Loop to continuously listen for the transform
    
    while (ros::ok()) {
        try {
            Eigen::Quaternionf q;
            Eigen::Vector3f t;
            Eigen::Matrix4f transformation;
            
            // Get the latest transform from source to target frame
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));

            q.x() = transformStamped.transform.rotation.x;
            q.y() = transformStamped.transform.rotation.y;
            q.z() = transformStamped.transform.rotation.z;
            q.w() = transformStamped.transform.rotation.w;

            t.x() = transformStamped.transform.translation.x;
            t.y() = transformStamped.transform.translation.y;
            t.z() = transformStamped.transform.translation.z; 

            transformation = transformation_matrix(t, q);
        
            return transformation;

        } catch (tf2::TransformException& ex) {
        }
    }
}

// Load tag pose in YAML file
YAML::Node get_tag_pose() {
  std::string dir = ros::package::getPath("broadcast_tf");
  dir = dir + "/config/tag_pose.yaml";

  try {
    YAML::Node file = YAML::LoadFile(dir);
    return file;
  }
  catch (...) {
    ROS_WARN("No tags file found");
  }
}

void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {

    Eigen::Quaternionf rot;
    Eigen::Vector3f trans;
    Eigen::Matrix4f tf_cam_to_tag;

    std::string tag_seen;

    int tag_id;
    
    for (int i=0; i < (msg->detections).size(); i++){

        tag_id = msg->detections[i].id[0];
        tag_seen = "tag_" + std::to_string(tag_id);
        
        trans.x() = msg->detections[i].pose.pose.pose.position.x;
        trans.y() = msg->detections[i].pose.pose.pose.position.y;
        trans.z() = msg->detections[i].pose.pose.pose.position.z;

        rot.x() = msg->detections[i].pose.pose.pose.orientation.x;
        rot.y() = msg->detections[i].pose.pose.pose.orientation.y;
        rot.z() = msg->detections[i].pose.pose.pose.orientation.z;
        rot.w() = msg->detections[i].pose.pose.pose.orientation.w;

        tf_cam_to_tag = transformation_matrix(trans, rot).inverse();

        try {
            YAML::Node data = get_tag_pose();

            Eigen::Quaternionf rot2;
            Eigen::Vector3f trans2;
            Eigen::Matrix4f tf_tag_to_map;

            trans2.x() = data[tag_seen]["x"].as<float>();
            trans2.y() = data[tag_seen]["y"].as<float>();
            trans2.z() = data[tag_seen]["z"].as<float>();

            rot2.x() = data[tag_seen]["qx"].as<float>();
            rot2.y() = data[tag_seen]["qy"].as<float>();
            rot2.z() = data[tag_seen]["qz"].as<float>();
            rot2.w() = data[tag_seen]["qw"].as<float>();

            tf_tag_to_map = transformation_matrix(trans2, rot2);

            Eigen::Matrix4f cam_pose = tf_tag_to_map * tf_cam_to_tag;
            Eigen::Matrix4f tf_tmp = tf_cam_to_base();

            Eigen::Matrix4f rb_pose = (cam_pose * tf_tmp);
    
            tf::Matrix3x3 orientation_matrix;
            orientation_matrix.setValue(rb_pose(0,0), rb_pose(0,1), rb_pose(0,2),
                                        rb_pose(1,0), rb_pose(1,1), rb_pose(1,2),
                                        rb_pose(2,0), rb_pose(2,1), rb_pose(2,2));
        
            // Getting the yaw value from the rotation matrix block in the homogenous transformation matrix.
            double roll, pitch, yaw;
            orientation_matrix.getRPY(roll, pitch, yaw);

            // Set quaternion message with only yaw value
            tf::Quaternion orientation_quat;
            orientation_quat.setRPY(0, 0, yaw);
            
            if (!published){
                geometry_msgs::PoseWithCovarianceStamped robot_pose;
                
                // Set up the message
                robot_pose.header.frame_id = "map";
                robot_pose.pose.pose.position.x = rb_pose(0,3);
                robot_pose.pose.pose.position.y = rb_pose(1,3);
                robot_pose.pose.pose.orientation.w = orientation_quat.getW();
                robot_pose.pose.pose.orientation.z = orientation_quat.getZ();
                robot_pose.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787};

                pose_pub.publish(robot_pose);
                current_tag = tag_seen;
                published = true;
            }
            else {
                ROS_INFO_ONCE("Initial pose published successfully");
                if (current_tag != tag_seen){
                    published = false;
                }
            }
        }
        catch (...) {
            ROS_ERROR_ONCE("Error occured when reading data. Check if tag id exist in database");
        }     
    }
}

int main(int argc, char** argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "transform_listener_node");
    ros::NodeHandle nh;
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
    ros::Subscriber tag_sub = nh.subscribe("/tag_detections",1 , callback);
    ros::spin();
    
    return 0;
}
