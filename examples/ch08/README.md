# Chapter 8: Unity Project Setup for Advanced Visualization

This directory contains the resources and instructions for setting up the Unity project to visualize the robot, as described in Chapter 8 of the book.

## Unity and ROS Integration

The connection between Unity and ROS 2 is achieved using the [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) package. This package provides two main components:

1.  **ROS-TCP-Endpoint**: A ROS 2 node that runs on the robot or in your ROS 2 workspace and acts as a bridge.
2.  **Unity Package**: A set of C# scripts and assets for your Unity project that communicate with the ROS-TCP-Endpoint.

## Setup Instructions

1.  **Create a new Unity Project**:
    - Open Unity Hub and create a new 3D project. Name it something descriptive, like `Robot-Visualization`.

2.  **Install the ROS-TCP-Connector Package**:
    - In your Unity project, navigate to `Window` > `Package Manager`.
    - Click the `+` icon in the top-left corner and select "Add package from git URL...".
    - Enter the following URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
    - Click "Add". Unity will download and install the package.

3.  **Configure ROS Settings**:
    - After the package is installed, a "Robotics" menu will appear in the Unity editor.
    - Go to `Robotics` > `ROS Settings`.
    - In the inspector window, you can configure the connection to your ROS network. For now, you can leave the `ROS IP Address` as `127.0.0.1` and the `ROS Port` as `10000`. This will connect to a ROS-TCP-Endpoint running on the same machine.

4.  **Import the Robot Model**:
    - The robot's visual model is defined in the URDF file from `examples/ch06/unitree_g1.urdf`.
    - To import this into Unity, you can use the [URDF-Importer](https://github.com/Unity-Technologies/URDF-Importer) package (install it via Git URL just like the ROS-TCP-Connector).
    - Once installed, you can use the `Assets` > `Import Robot from URDF` menu to import and configure the `unitree_g1.urdf` file.

## Next Steps

With the project configured, you are now ready to create C# scripts that subscribe to ROS 2 topics (like `/joint_states` and `/tf`) to animate the robot model in real-time. This process is detailed in the main text of Chapter 8.
