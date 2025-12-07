# Chapter 11: Isaac ROS AprilTag Pipeline Example

This directory contains a ROS 2 launch file (`apriltag_pipeline.launch.py`) for setting up an AprilTag detection pipeline using the Isaac ROS GEM.

## About the Launch File

The `apriltag_pipeline.launch.py` script is designed to demonstrate the proper way to run a hardware-accelerated Isaac ROS GEM. It does the following:

1.  **Declares a Launch Argument**: It allows the user to specify the physical size of the AprilTags being used, which is critical for accurate pose estimation.
2.  **Defines a Composable Node**: It configures the `AprilTagNode` from the `isaac_ros_apriltag` package. It sets parameters like the tag size and which processing backend to use (CUDABACKEND for GPU acceleration).
3.  **Sets Topic Remappings**: It tells the node to listen for images on `/robot/image_raw` and camera information on `/robot/camera_info`, which are the topics our simulated camera will publish on.
4.  **Starts a Node Container**: It launches a `component_container` process, which is the standard ROS 2 way to host composable nodes.
5.  **Loads the Node into the Container**: It loads the configured `AprilTagNode` into the container, enabling high-performance, intra-process communication.

## How to Run This Example

1.  **Prerequisites**:
    - You must have Isaac Sim running with a camera that is publishing images to the `/robot/image_raw` topic. The `basic_scene.py` from Chapter 10 can be adapted for this.
    - You must have the `isaac_ros_apriltag` package installed in your ROS 2 workspace (e.g., via `sudo apt-get install ros-humble-isaac-ros-apriltag`).
    - You need to have an AprilTag visible to the camera in your Isaac Sim environment.

2.  **Source your ROS 2 Workspace**:
    ```bash
    # Replace ~/ros2_ws with the path to your workspace
    source ~/ros2_ws/install/setup.bash 
    ```

3.  **Run the Launch File**:
    Navigate to the root of this project directory in your terminal and execute the following command:
    ```bash
    ros2 launch examples/ch11/apriltag_pipeline.launch.py tag_size:=0.10
    ```
    - The `tag_size` argument should be set to the actual size (in meters) of the AprilTag you placed in your simulation.

4.  **Verify the Output**:
    You can check that the pipeline is working by visualizing the output in RViz2 or by checking the topics.
    - **Check Topics**:
      ```bash
      ros2 topic list
      # You should see /tag_detections in the list
      ```
    - **Echo Detections**:
      ```bash
      ros2 topic echo /tag_detections
      # If an AprilTag is visible, this will print pose information.
      ```
    - **Check TF Frames**:
      The `isaac_ros_apriltag` node also publishes a TF frame for each detected tag. You can check this with:
      ```bash
      ros2 run tf2_ros tf2_echo <camera_frame> <tag_frame>
      # Example: ros2 run tf2_ros tf2_echo head_camera_link apriltag_36h11_0
      ```
