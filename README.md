<h1>Pose-Estimating-Using-AprilTag package for ROS</h1>

1. `config` folder contain the `tag_pose.yaml` file for storing the tag pose in the global map frame (This is achived by running the `record_tag_pose.py`)
2. `src` folder contain the scripts for each task.
    - **Recording the tag pose in a predefined map as an initial setup** (`record_tag_pose.py`)
    - **Publish initial pose estimation when tags are in sight. When seeing new tags, initial pose is republished** (`pose_estimate.py`)
    - **Set goal for robot base on the tag location (currently working on this)** (`set_goal.py`) 
3. Note on running `record_tag_pose.py`
    At the initial setup, the robot should know its location on the map so that the tag pose can be stored accurately. This can be done by giving hint for the robot when first setup the robot.
4. Change the path to the `tag_pose.yaml` file in each scripts before running in order to work.# Pose-Estimation-Using-AprilTag

<h2>Dependencies</h2>

This package require the `apriltag_ros` package which can be build from [here](https://github.com/AprilRobotics/apriltag_ros)

<h2>Nodes</h2>

`pose_estimate.py` publish the initial pose via the topic `/initialpose`
