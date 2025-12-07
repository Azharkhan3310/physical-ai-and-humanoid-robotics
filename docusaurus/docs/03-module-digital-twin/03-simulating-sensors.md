---
id: simulating-sensors-and-physics
sidebar_position: 3
title: Simulating Sensors and Physics
---

# Chapter 9: Simulating Sensors and Physics

A digital twin is more than just a visual model; it must also behave like its real-world counterpart. This requires accurate simulation of both its physical properties and its sensors. Gazebo, in conjunction with ROS 2, provides a powerful framework for modeling these complex characteristics.

This chapter explores how to enhance our robot's SDF/URDF file to include realistic physics and a suite of common sensors.

## The Importance of Accurate Simulation

1.  **Physics Simulation**: Accurate mass, inertia, and collision properties are essential for the simulator to correctly model how the robot moves, interacts with its environment, and responds to forces. This is critical for testing walking gaits, manipulation, and stability.
2.  **Sensor Simulation**: Realistic sensor data is the foundation of the robot's perception pipeline. By simulating sensors like LiDAR, cameras, and IMUs, we can develop and test algorithms for localization, mapping, object detection, and navigation entirely in simulation before deploying to hardware.

## Adding Physics Properties to a URDF/SDF

Within each `<link>` element of a URDF or SDF file, we define its physical characteristics using three main tags: `<inertial>`, `<visual>`, and `<collision>`.

-   **`<visual>`**: Defines how the link looks. This is what you see in the simulator.
-   **`<collision>`**: Defines the link's collision geometry, which is a (usually simplified) shape used by the physics engine to calculate contact forces.
-   **`<inertial>`**: Defines the link's dynamic properties: its mass and inertia matrix.

**Example: Inertial properties for a robot arm link**

```xml
<link name="upper_arm_link">
  <inertial>
    <origin xyz="0 0 0.05" rpy="0 0 0" />
    <mass value="1.2" />
    <inertia 
      ixx="0.01" ixy="0.0" ixz="0.0" 
      iyy="0.01" iyz="0.0" 
      izz="0.005" />
  </inertial>

  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.2" />
    </geometry>
  </visual>
  
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.2" />
    </geometry>
  </collision>
</link>
```

-   `<mass>` is specified in kilograms.
-   `<inertia>` is the 3x3 inertia tensor, representing the link's resistance to angular acceleration.

## Adding Sensor Plugins in Gazebo

Gazebo uses its own simulation description format (SDF) to add rich details like sensors. You can embed SDF directly within a URDF file using the `<gazebo>` tag.

The most common approach is to attach a sensor to a specific `<link>` on the robot.

### 1. Camera Sensor

This example attaches a camera to the robot's `head_link`.

```xml
<gazebo reference="head_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.396</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <argument>--ros-args -r image_raw:=image_raw</argument>
        <argument>--ros-args -r camera_info:=camera_info</argument>
      </ros>
      <camera_name>head_camera</camera_name>
      <frame_name>head_camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

-   **`libgazebo_ros_camera.so`**: This is the crucial Gazebo plugin that simulates a camera and publishes its images to a ROS 2 topic.
-   **`<ros>` block**: Configures the ROS 2 interface, including the topic name (`/robot/image_raw`).

### 2. 2D LiDAR Sensor

This example adds a 2D LiDAR for navigation and mapping.

```xml
<gazebo reference="base_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0.1 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.57</min_angle>
          <max_angle>1.57</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <argument>--ros-args -r out:=scan</argument>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```
-   **`libgazebo_ros_ray_sensor.so`**: The plugin for laser-based sensors.
-   It publishes `sensor_msgs/msg/LaserScan` messages to the `/robot/scan` topic.

### 3. IMU Sensor

An IMU is vital for state estimation.

```xml
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
          <namespace>/robot</namespace>
          <remapping>~/out:=imu/data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```
-   **`libgazebo_ros_imu_sensor.so`**: The plugin for simulating an IMU.
-   It publishes `sensor_msgs/msg/Imu` messages to `/robot/imu/data`.

## Example URDF with Sensors

An updated URDF file for the Unitree G1 model, including all the sensor plugins discussed in this chapter (Camera, LiDAR, and IMU), has been created as a reference. You can view it or download it from the examples directory.

[**Chapter 9 Example: unitree_g1_sensors.urdf**](../../../examples/ch09/unitree_g1_sensors.urdf)

## Next Steps

By adding these plugins to your robot's description file, you create a high-fidelity digital twin whose sensor data can be used directly by ROS 2 packages like `slam_toolbox` (for mapping), `robot_localization` (for state estimation), and your own custom perception nodes. In the next module, we will begin using these simulated sensors to give our robot intelligence.
