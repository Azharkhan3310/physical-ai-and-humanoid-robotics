---
id: navigation-nav2
sidebar_position: 3
title: Navigation with Nav2
---

# Chapter 12: Navigation with Nav2

Once a robot can perceive its environment, the next logical step is to navigate it autonomously. The de facto standard for robotic navigation in the ROS 2 ecosystem is the **Navigation2 stack**, commonly known as Nav2.

This chapter provides an overview of the Nav2 stack and discusses how to configure it to work with our simulated robot in Isaac Sim.

## What is Nav2?

Nav2 is the second generation of the ROS Navigation Stack. It is a powerful, flexible, and fully-featured system that enables a robot to move from a starting point to a destination safely and efficiently. It is not a single node but rather a collection of servers, lifecycle managers, and controllers that work together.

### Key Components of Nav2

The Nav2 stack can be broken down into several key functionalities:

1.  **Localization (AMCL)**: Before a robot can navigate, it must first know where it is. Nav2 typically uses the **Adaptive Monte Carlo Localization (AMCL)** server to estimate the robot's position and orientation on a pre-existing map. AMCL takes in laser scan data (`/scan`) and odometry data (`/odom`) to maintain this estimate.

2.  **Global Planning**: When given a goal, the **Planner Server** creates a high-level path from the robot's current location to the destination. It uses a costmap, which represents obstacles, to find the optimal path.

3.  **Local Planning and Control**: The **Controller Server** is responsible for executing the global plan. It generates velocity commands (`/cmd_vel`) to send to the robot's base, while reacting to immediate obstacles that may not have been on the global costmap. This is often called local planning or obstacle avoidance.

4.  **Behaviors**: The **Behavior Server** orchestrates complex navigation tasks that go beyond simple A-to-B movement, such as spinning in place to clear the costmap or backing up.

5.  **Costmaps**: Nav2 uses two main costmaps:
    -   The **Global Costmap** is used by the planner for the long-range path.
    -   The **Local Costmap** is a smaller, rolling window around the robot used by the controller for immediate obstacle avoidance.

![Nav2 Architecture Diagram](https://navigation.ros.org/_images/columbia.png)
*Image credit: ROS 2 Navigation Working Group*

## Configuring Nav2 for Isaac Sim

To get Nav2 running with our Isaac Sim robot, we need to provide it with three main things:

1.  **A Map of the World**: You can generate a map of your Isaac Sim environment using the `slam_toolbox` package, which listens to `/scan` and `/tf` data as you drive the robot around. This produces a `map.yaml` and `map.pgm` file.

2.  **Correctly Published Topics**: Nav2 expects several topics to be available, all of which can be provided by our simulated robot and sensor plugins:
    -   `/scan` (`sensor_msgs/msg/LaserScan`): From our simulated LiDAR.
    -   `/odom` (`nav_msgs/msg/Odometry`): Provided by Isaac Sim's physics engine.
    -   `/tf` and `/tf_static` (`tf2_msgs/msg/TFMessage`): To describe the relationship between all the robot's links (e.g., `base_link` -> `lidar_link`).

3.  **A Nav2 Configuration File**: A YAML file (`nav2_params.yaml`) is used to configure all the servers and plugins within Nav2. This is where you specify things like robot dimensions, maximum speed, costmap inflation radius, and which planner or controller plugins to use.

### Example Launch File

Bringing up the entire Nav2 stack is typically done with a master launch file.

```python
# Example Nav2 launch file snippet

def generate_launch_description():
    # Path to your map file
    map_file = LaunchConfiguration('map')
    # Path to your Nav2 params file
    params_file = LaunchConfiguration('params_file')
    
    # ... (code to declare launch arguments for map and params)

    # Start the Nav2 lifecycle manager
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescription(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file
        ].items(),
    )

    return LaunchDescription([
        # ... (your launch arguments)
        nav2_bringup_cmd
    ])
```

## Example: Nav2 Configuration

A full set of example files for launching and configuring Nav2 for our simulated robot has been created for this chapter. This includes a parameters file and a top-level launch script.

Full instructions on how to generate a map and run the launch file are available in the README.

-   [**Chapter 12 Example: README**](../../../examples/ch12/README.md)
-   [**Chapter 12 Example: Launch File**](../../../examples/ch12/nav2_bringup.launch.py)
-   [**Chapter 12 Example: Parameters File**](../../../examples/ch12/nav2_params.yaml)

## Next Steps

With Nav2 configured, you can send navigation goals to your robot from RViz2, from the command line, or, most importantly for this book, from another ROS 2 node. In the next module, we will create a "brain" node that uses an LLM to generate these navigation goals based on natural language commands.
