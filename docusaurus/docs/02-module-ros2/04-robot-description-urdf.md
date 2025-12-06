---
id: robot-description-urdf
sidebar_position: 4
title: Describing the Robot (URDF/SDF)
---

# Describing the Robot (URDF/SDF)

For a robot to be controlled and simulated effectively, its physical characteristics must be accurately described. In ROS 2, the Unified Robot Description Format (URDF) and its successor, Semantic Robot Description Format (SRDF), are XML-based file formats used to describe a robot's kinematic and dynamic properties, visual appearance, and collision models. This chapter will guide you through creating a URDF model for our humanoid robot.

## What is URDF?

URDF is an XML file format that describes a robot's:
*   **Links**: The rigid bodies of the robot (e.g., torso, upper arm, forearm).
*   **Joints**: The connections between links, defining their type (revolute, prismatic, fixed) and movement limits.
*   **Visuals**: The 3D models and colors that represent the robot visually.
*   **Collisions**: Simplified 3D models used for collision detection in simulation.
*   **Inertials**: Mass and inertia properties of links, crucial for realistic physics simulation.

URDF is hierarchical, with a single root link and subsequent links connected by joints.

## Creating a Simple URDF

Let's start by creating a very simple robot description.

1.  **Create a Package for Robot Descriptions**:
    It's good practice to keep robot descriptions in a dedicated package.
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake robot_description
    ```
    We use `ament_cmake` because URDF files are typically used with C++-based tools and often involve CMake build processes.

2.  **Create Your First URDF File**:
    Inside `~/ros2_2_ws/src/robot_description/urdf/`, create a file named `my_simple_robot.urdf`.
    ```xml
    <?xml version="1.0"?>
    <robot name="my_simple_robot">

      <link name="base_link">
        <visual>
          <geometry>
            <box size="0.6 0.4 0.2"/>
          </geometry>
          <material name="blue">
            <color rgba="0 0 0.8 1"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <box size="0.6 0.4 0.2"/>
          </geometry>
        </collision>
        <inertial>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <mass value="20.0"/>
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
      </link>

      <link name="arm_link">
        <visual>
          <geometry>
            <cylinder radius="0.05" length="0.5"/>
          </geometry>
          <origin xyz="0 0 0.25" rpy="0 0 0"/>
          <material name="red">
            <color rgba="0.8 0 0 1"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <cylinder radius="0.05" length="0.5"/>
          </geometry>
          <origin xyz="0 0 0.25" rpy="0 0 0"/>
        </collision>
        <inertial>
          <origin xyz="0 0 0.25" rpy="0 0 0"/>
          <mass value="2.0"/>
          <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
        </inertial>
      </link>

      <joint name="base_to_arm_joint" type="revolute">
        <parent link="base_link"/>
        <child link="arm_link"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
      </joint>

      <material name="blue"/>
      <material name="red"/>

    </robot>
    ```

3.  **Visualize with `rviz2`**:
    To visualize your URDF model, you can use `rviz2`.
    ```bash
    # In one terminal, launch the URDF publisher
    ros2 launch urdf_tutorial display.launch.py model:=src/robot_description/urdf/my_simple_robot.urdf.xacro # (if using xacro)
    # or just use
    ros2 run rviz2 rviz2
    # In rviz2, add "RobotModel" and select your URDF from the "Description File"
    ```

## Describing a Humanoid Robot (Unitree G1/G2)

For our humanoid robot, the URDF will be significantly more complex, involving many links and joints for the torso, head, arms, and legs. We will typically use `xacro` (XML Macros) to manage this complexity, allowing for more modular and readable robot descriptions.

The `robot_description` package will contain:
*   `urdf/`: URDF and XACRO files for the humanoid robot.
*   `meshes/`: 3D models (e.g., `.stl`, `.dae`) referenced by the URDF for visual and collision properties.
*   `launch/`: Launch files to spawn the robot in simulation and `rviz2`.

## Conclusion

URDF and XACRO are essential tools for defining your robot's physical structure for both simulation and control. A well-designed robot description is a prerequisite for accurate kinematics, dynamics, and visualization. In the next module, we will delve into building immersive digital twin environments using Gazebo and Unity.
