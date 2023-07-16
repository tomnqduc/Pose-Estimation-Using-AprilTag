#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <tf/tf.h>

ros::Publisher goal_pub;

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

void goal_publisher(char* tag_id[]){
    YAML::Node data = get_tag_pose();

    // Se goal to be 75cm from the tag
    Eigen::Vector3f trans_goal;
    Eigen::Quaternionf rot_goal;

    trans_goal.x() = 0;
    trans_goal.y() = 0;
    trans_goal.z() = 1;

    rot_goal.x() = 0;
    rot_goal.y() = 0.7071068;
    rot_goal.z() = 0;
    rot_goal.w() = 0.7071068;

    Eigen::Matrix4f tf_goal_to_tag = transformation_matrix(trans_goal, rot_goal);

    geometry_msgs::PoseStamped goal_msg;

    if (data[tag_id[1]]) {
        Eigen::Vector3f t;
        Eigen::Quaternionf r;

        t.x() = data[tag_id[1]]["x"].as<float>();
        t.y() = data[tag_id[1]]["y"].as<float>();
        t.z() = data[tag_id[1]]["z"].as<float>();

        r.x() = data[tag_id[1]]["qx"].as<float>();
        r.y() = data[tag_id[1]]["qy"].as<float>();
        r.z() = data[tag_id[1]]["qz"].as<float>();
        r.w() = data[tag_id[1]]["qw"].as<float>();

        Eigen::Matrix4f tf_tag_to_map = transformation_matrix(t, r);

        Eigen::Matrix4f goal = tf_tag_to_map * tf_goal_to_tag;

        tf::Matrix3x3 orientation;
        orientation.setValue(goal(0,0), goal(0,1), goal(0,2),
                             goal(1,0), goal(1,1), goal(1,2),
                             goal(2,0), goal(2,1), goal(2,2));
        double roll, pitch, yaw;
        orientation.getRPY(roll, pitch, yaw);

        tf::Quaternion orientation_quat;
        orientation_quat.setRPY(0, 0, yaw);

        // Declare the message
        
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = goal(0,3);
        goal_msg.pose.position.y = goal(1,3);
        goal_msg.pose.orientation.w = orientation_quat.w();
        goal_msg.pose.orientation.z = orientation_quat.z();

        
        goal_pub.publish(goal_msg);        
    }
    else {
        ROS_WARN_ONCE("Tag not found in map");
    }
}

int main(int argc, char* argv[]){
    
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle nh;
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
      
    int i=0;
    ros::Rate rate(10);
    while (i<2) {
        goal_publisher(argv);
        rate.sleep();
        i++;
    }

    return 0;
}
