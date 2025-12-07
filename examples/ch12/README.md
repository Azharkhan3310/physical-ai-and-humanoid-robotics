# Chapter 12: Nav2 Configuration and Launch Examples

This directory contains example files for configuring and launching the Nav2 stack for our simulated robot.

## Files in this Directory

1.  **`nav2_params.yaml`**:
    -   This is a ROS 2 parameters file that configures the various nodes and servers within the Nav2 stack.
    -   It defines settings for localization (AMCL), the global and local planners, the controller, and the costmaps.
    -   **Note**: This is a simplified configuration. A production setup would require more fine-tuning based on the specific robot and environment.

2.  **`nav2_bringup.launch.py`**:
    -   This is a top-level ROS 2 launch file used to start the entire Nav2 stack.
    -   It acts as a wrapper around the default `bringup_launch.py` from the `nav2_bringup` package.
    -   Its main purpose is to pass our custom `nav2_params.yaml` file to the Nav2 system.

## How to Run This Example

Launching the Nav2 stack requires that all the prerequisite nodes and topics are running.

1.  **Prerequisites**:
    -   An instance of Isaac Sim (or Gazebo) must be running with the robot loaded.
    -   The robot simulation must be publishing valid `/tf` transforms between all its links (e.g., `odom` -> `base_link` -> `lidar_link`).
    -   A LiDAR sensor must be publishing `/scan` messages.
    -   A localization node (like AMCL, which is started by this launch file) or another odometry source must be providing the `map` -> `odom` transform.

2.  **Generate a Map**:
    -   Before you can navigate, you need a map. You can create one using a SLAM package like `slam_toolbox`.
    -   `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true`
    -   Drive the robot around the environment in Isaac Sim to build the map.
    -   Save the map using the `nav2_map_server` tools:
        ```bash
        ros2 run nav2_map_server map_saver_cli -f my_map
        ```
    -   This will create `my_map.yaml` and `my_map.pgm`. You should place these in a `maps` subdirectory within this example folder. The launch file assumes this location by default.

3.  **Run the Launch File**:
    -   Source your ROS 2 workspace.
    -   Execute the launch file from the root of this project:
        ```bash
        ros2 launch examples/ch12/nav2_bringup.launch.py
        ```
    -   This command will start all the Nav2 servers, load the map, and initialize the robot's localization.

4.  **Send a Navigation Goal**:
    -   Open RViz2 and configure it to display the map, robot model, and costmaps.
        ```bash
        ros2 launch nav2_bringup rviz_launch.py
        ```
    -   Use the "Nav2 Goal" tool in the RViz2 toolbar to click on the map and set a destination for the robot.
    -   The robot should begin planning a path and moving towards the goal.
