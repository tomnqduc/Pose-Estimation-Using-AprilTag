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

- `pose_estimate.py` and `pose_estimate`(C++ node) publish the initial pose via the topic `/initialpose` and get subscibed by AMCL algorithm.
- `set_goal.py` publish to the topic `/move_base_simple/goal` which usually published by click and drag on RViz.

<h2> Workflow and Setup </h2>
<p>
    <img src="https://github.com/tomnqduc/Pose-Estimation-Using-AprilTag/assets/86122117/d18d9386-c35a-4e90-bdbb-25d138ea9515" />
    <img src="https://github.com/tomnqduc/Pose-Estimation-Using-AprilTag/assets/86122117/0d3430d8-07aa-43bb-b523-a08a1911d7a5" />
</p>

<h2>Demo result</h2>

The robot intitially have no clue about its position on the map, but it can see the tag. The tag will act as a hint for the robot to relocalize and set its initial pose for the AMCL algorithm.

https://github.com/tomnqduc/Pose-Estimation-Using-AprilTag/assets/86122117/036f9ce0-acff-48a9-aaaa-6be95a6dc307


![]
