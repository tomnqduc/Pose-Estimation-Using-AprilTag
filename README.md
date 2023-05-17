<h1>Pose-Estimating-Using-AprilTag package for ROS</h1>

1. `config` folder contain the `tag_pose.yaml` file for storing the tag pose in the global map frame (This is achived by running the `record_tag_pose.py`)
2. `scripts` folder contain the python scripts for each task.
    - **Recording the tag pose in a predefined map as an initial setup** (`record_tag_pose.py`)
    - **Publish initial pose estimation when tags are in sight. When seeing new tags, initial pose is republished** (`pose_estimate.py`)
    - **Set goal for robot base on the tag location (currently working on this)** (`set_goal.py`) 
3. Note on running `record_tag_pose.py`
    At the initial setup, the robot should know its location on the map so that the tag pose can be stored accurately. This can be done by giving hint for the robot when first setup the robot.
4. Path for the `tag_pose.yaml` file now can automatically locate if the workspace is source properly.
5. Update the new C++ node `pose_estimate` for faster process. To run, type in terminal `rosrun broadcast_tf pose_estimate`.

<h2>Dependencies</h2>

- This package require the `apriltag_ros` package which can be build from [here](https://github.com/AprilRobotics/apriltag_ros)
- `Eigen` from C++ also require to build the C++ node. Installation guide can be found [here](https://eigen.tuxfamily.org/dox/GettingStarted.html)

<h2>Nodes</h2>

`pose_estimate.py` publish the initial pose via the topic `/initialpose`
