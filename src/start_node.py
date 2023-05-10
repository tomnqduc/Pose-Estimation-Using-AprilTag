#!/usr/bin/env python 
import roslaunch
import rospy
import rosnode

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/tomnq/Desktop/AT/AT_ws/src/apriltag_ros/apriltag_ros/launch/continuous_detection.launch"])
launch.start()
rospy.loginfo("started")
print(launch)
print(rosnode.get_node_names())
launch.spin()

