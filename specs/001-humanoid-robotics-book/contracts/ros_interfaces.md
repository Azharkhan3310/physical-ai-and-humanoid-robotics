# ROS 2 Interface Contracts

This document defines the key ROS 2 interfaces (topics, services, actions) that form the communication backbone of the humanoid robot project.

---

## Standard Interfaces (from common_interfaces)

The project will heavily leverage standard, community-accepted interfaces to ensure interoperability.

### `geometry_msgs/msg/Twist`
- **Topic**: `/cmd_vel`
- **Purpose**: To send linear and angular velocity commands to the robot's base for navigation.
- **Publisher**: Navigation Stack (Nav2)
- **Subscriber**: Robot's base controller (e.g., `diff_drive_controller`)

### `sensor_msgs/msg/LaserScan`
- **Topic**: `/scan`
- **Purpose**: To publish 2D LiDAR data for mapping and localization.
- **Publisher**: LiDAR driver or Gazebo LiDAR sensor plugin.
- **Subscriber**: SLAM / Localization node (e.g., `slam_toolbox`).

### `sensor_msgs/msg/Image`
- **Topic**: `/camera/color/image_raw`, `/camera/depth/image_raw`
- **Purpose**: To publish color and depth images from the robot's camera (RealSense D435i).
- **Publisher**: RealSense camera driver or Isaac Sim camera sensor.
- **Subscriber**: Perception nodes (e.g., AprilTag detection, object recognition).

### `sensor_msgs/msg/Imu`
- **Topic**: `/imu/data`
- **Purpose**: To publish data from the Inertial Measurement Unit for orientation and state estimation.
- **Publisher**: IMU driver or Gazebo IMU sensor plugin.
- **Subscriber**: Robot Localization nodes (`robot_localization`).

### `trajectory_msgs/msg/JointTrajectory`
- **Topic**: `/joint_trajectory_controller/joint_trajectory`
- **Purpose**: To send a sequence of joint positions for controlling the robot's arms or legs.
- **Publisher**: Motion Planning (MoveIt 2) or custom action server.
- **Subscriber**: `joint_trajectory_controller`.

---

## Custom Action Interfaces

Custom actions will be defined for high-level, goal-oriented tasks, particularly for the VLA module.

### `robot_interfaces/action/ExecuteTask`

- **Purpose**: This action allows the VLA planner to send a high-level task to the robot's control system. It provides a structured way to request an action and receive feedback.
- **Goal**:
  - `string task_name` (e.g., "navigate_to", "pick_up", "wave_hand")
  - `string[] task_parameters` (e.g., `["kitchen"]`, `["red_block"]`, `[]`)
- **Result**:
  - `bool success`
  - `string message`
- **Feedback**:
  - `string status` (e.g., "Executing navigation", "Object detected", "Manipulation failed")

### Example Usage (VLA to Action Server):
1.  **VLA Planner** receives voice command: "Go to the kitchen".
2.  **VLA Planner** translates this to an `ExecuteTask` goal:
    - `task_name: "navigate_to"`
    - `task_parameters: ["kitchen"]`
3.  **VLA Planner** sends the goal to the `/execute_task` action server.
4.  The **Action Server** (a custom ROS 2 node) receives the goal, calls the Nav2 API to navigate, and provides feedback (`"Executing navigation"`).
5.  Once navigation is complete, the **Action Server** returns the result (`success: true`).

---

### `robot_interfaces/srv/GetKnownLocations`

- **Purpose**: A service to allow the VLA planner to query the robot's "memory" of named locations in its map.
- **Request**:
  - `(empty)`
- **Response**:
  - `string[] locations` (e.g., `["kitchen", "charging_dock", "table"]`)

---
*Note: The `robot_interfaces` package will need to be created to contain these custom message definitions.*
