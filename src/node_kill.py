#!/usr/bin/env python 

import rospy
import rosnode
from geometry_msgs.msg import PoseWithCovarianceStamped

class NodeKiller():
    def __init__(self):
        self.killer = rospy.init_node("node_killer")
        self.node_running = rosnode.get_node_names()
        self.initial_pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.callback)
        

    def callback(self, data):
        seq = data.header.seq

        # Specify nodes to be killed
        node_to_be_kill = ["/apriltag_ros_continuous_node", "/initial_pose_publisher"]

        # Print all running nodes
        print(self.node_running)
        
        if seq == 10 and "/apriltag_ros_continuous_node" in self.node_running:
            rosnode.kill_nodes(node_to_be_kill)
            
        

if __name__ == "__main__":
    node_kill = NodeKiller()
    rospy.spin()
