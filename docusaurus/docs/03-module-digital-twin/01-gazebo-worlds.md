---
id: gazebo-worlds
sidebar_position: 1
title: Building Simulation Worlds (Gazebo)
---

# Building Simulation Worlds (Gazebo)

Simulation is a critical tool in robotics development, allowing for rapid prototyping, testing, and debugging without the constraints and risks of real-world hardware. Gazebo, now often referred to as Ignition Gazebo, is a powerful 3D robotics simulator that integrates seamlessly with ROS 2. In this chapter, you will learn how to create and customize simulation worlds, spawn robot models, and interact with the simulated environment.

## Introduction to Gazebo (Ignition)

Gazebo provides the ability to accurately simulate populations of robots in complex indoor and outdoor environments. It offers a robust physics engine, high-quality graphics, and convenient programmatic interfaces.

### Key Components:
*   **World Files (`.sdf`)**: XML files that define the environment, including terrain, objects, light sources, and physics properties.
*   **Model Files (`.sdf` or `.urdf`)**: XML files that describe individual robots or objects within the world.
*   **Plugins**: Extend Gazebo's functionality, allowing custom sensor integration, controller interfaces, and more.

## Creating a Simple Gazebo World

Let's create a basic world with a flat ground plane and a light source.

1.  **Create a Package for Gazebo Worlds**:
    It's good practice to organize your simulation assets.
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake my_gazebo_worlds
    ```

2.  **Create Your First World File**:
    Inside `~/ros2_ws/src/my_gazebo_worlds/worlds/`, create a file named `my_empty_world.sdf`.
    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.7">
      <world name="my_empty_world">
        <light name="sun" type="directional">
          <cast_shadows>1</cast_shadows>
          <pose>0 0 10 0 -0 0</pose>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
          </attenuation>
          <direction>-0.5 0.1 -0.9</direction>
        </light>
        <model name="ground_plane">
          <static>true</static>
          <link name="link">
            <collision name="collision">
              <geometry>
                <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
                </plane>
              </geometry>
              <surface>
                <friction>
                  <ode>
                    <mu>1.0</mu>
                    <mu2>1.0</mu2>
                  </ode>
                </friction>
              </surface>
            </collision>
            <visual name="visual">
              <geometry>
                <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
                </plane>
              </geometry>
              <material>
                <ambient>0.8 0.8 0.8 1</ambient>
                <diffuse>0.8 0.8 0.8 1</diffuse>
                <specular>0.8 0.8 0.8 1</specular>
              </material>
            </visual>
          </link>
        </model>
      </world>
    </sdf>
    ```
    This file defines a `world` named `my_empty_world` with a `sun` light source and a `ground_plane` model.

3.  **Launch the World**:
    ```bash
    gazebo --verbose -r my_empty_world.sdf
    ```
    *Expected Output:*
    A Gazebo window should open showing a flat gray plane and directional lighting.

## Spawning Your Robot Model

Now, let's spawn the URDF model of our humanoid robot (from Chapter 6) into this world.

1.  **Update `CMakeLists.txt`**: Ensure your `my_gazebo_worlds` package's `CMakeLists.txt` is configured to install the world file.
    ```cmake
    # ...
    install(DIRECTORY worlds
      DESTINATION share/${PROJECT_NAME}
    )
    ```

2.  **Create a Launch File**: In `~/ros2_ws/src/my_gazebo_worlds/launch/`, create `spawn_robot_world.launch.py`.
    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    def generate_launch_description():
        pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
        pkg_robot_description = get_package_share_directory('robot_description') # Assuming robot_description package exists

        # Path to our custom world file
        world_file_name = 'my_empty_world.sdf'
        world_path = os.path.join(get_package_share_directory('my_gazebo_worlds'), 'worlds', world_file_name)

        # Launch Gazebo
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
        )

        # Robot description
        urdf_file_name = 'unitree_g1.urdf' # Assuming this is in robot_description/urdf
        urdf_path = os.path.join(pkg_robot_description, 'urdf', urdf_file_name)

        # Robot State Publisher node
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf_path}],
            arguments=[urdf_path],
        )

        # Spawn the robot into Gazebo
        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'unitree_g1', '-file', urdf_path],
            output='screen'
        )

        return LaunchDescription([
            gazebo_launch,
            robot_state_publisher_node,
            spawn_entity_node,
        ])
    ```

3.  **Build and Launch**:
    ```bash
    cd ~/ros2_ws/
    colcon build --packages-select my_gazebo_worlds robot_description
    source install/setup.bash
    ros2 launch my_gazebo_worlds spawn_robot_world.launch.py
    ```
    *Expected Output:*
    A Gazebo window should open with your URDF robot model spawned in the world.

## Conclusion

Creating custom Gazebo worlds and spawning ROS 2 robot models within them are foundational skills for robotics simulation. This allows you to test your robot's behavior in a controlled virtual environment. In the next chapter, we will explore how to visualize these simulations using Unity.
