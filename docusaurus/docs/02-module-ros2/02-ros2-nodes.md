---
id: ros2-nodes
sidebar_position: 2
title: Creating ROS 2 Nodes (rclpy)
---

# Creating ROS 2 Nodes (rclpy)

In ROS 2, `nodes` are the fundamental building blocks of a robot application. Each node is responsible for a single, well-defined task. In this chapter, you will learn how to create, compile, and run ROS 2 nodes using `rclpy`, the Python client library.

## Setting Up Your ROS 2 Workspace

Before creating your first node, you need a ROS 2 workspace. A workspace is a directory where you can develop your own ROS 2 packages.

1.  **Create a Workspace Directory**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```
    This creates a directory `ros2_ws` and a sub-directory `src` inside it. Your ROS 2 packages will reside in `src`.

2.  **Source ROS 2 Setup**: Ensure your ROS 2 environment is sourced.
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    If you added this to your `~/.bashrc` in Chapter 2, you just need to open a new terminal.

## Creating a ROS 2 Package

A ROS 2 package is a container for your nodes, launch files, and other related resources.

1.  **Create a New Package**: Navigate into your workspace's `src` directory and create a new package called `my_robot_pkg`.
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python my_robot_pkg
    ```
    This command creates a new directory `my_robot_pkg` with a basic structure for a Python package.

2.  **Explore the Package Structure**:
    ```
    my_robot_pkg/
    ├── my_robot_pkg/             # Python source files go here
    │   └── __init__.py
    ├── resource/
    │   └── my_robot_pkg          # Used for ament_index_python
    ├── setup.py                  # Python package build configuration
    ├── package.xml               # Package metadata (dependencies, etc.)
    └── setup.cfg                 # Python package configuration
    ```

## Writing Your First Node: A Simple Publisher

Let's create a simple "talker" node that publishes a "Hello World" message to a topic.

1.  **Create a Python File**: Inside `~/ros2_ws/src/my_robot_pkg/my_robot_pkg/`, create a new file named `talker_node.py`.
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimpleTalker(Node):
        def __init__(self):
            super().__init__('simple_talker')
            self.publisher_ = self.create_publisher(String, 'chatter', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = f'Hello World: {self.i}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1

    def main(args=None):
        rclpy.init(args=args)
        simple_talker = SimpleTalker()
        rclpy.spin(simple_talker)
        simple_talker.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2.  **Make it Executable**:
    ```bash
    chmod +x ~/ros2_ws/src/my_robot_pkg/my_robot_pkg/talker_node.py
    ```

3.  **Update `setup.py`**: Modify `setup.py` to tell ROS 2 how to find and execute your node.
    - Add `import os` and `from glob import glob`
    - Add your Python script to the `entry_points` dictionary under `console_scripts`:
    ```python
    # ... other imports
    import os
    from glob import glob
    # ...

    setup(
        # ... other parameters
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yem]'))),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='your_name',
        maintainer_email='your_email@example.com',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'talker = my_robot_pkg.talker_node:main',
            ],
        },
    )
    ```

## Writing Your Second Node: A Simple Subscriber

Now let's create a "listener" node that subscribes to the `chatter` topic and prints the messages it receives.

1.  **Create a Python File**: Inside `~/ros2_ws/src/my_robot_pkg/my_robot_pkg/`, create a new file named `listener_node.py`.
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimpleListener(Node):
        def __init__(self):
            super().__init__('simple_listener')
            self.subscription = self.create_subscription(
                String,
                'chatter',
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning

        def listener_callback(self, msg):
            self.get_logger().info(f'I heard: "{msg.data}"')

    def main(args=None):
        rclpy.init(args=args)
        simple_listener = SimpleListener()
        rclpy.spin(simple_listener)
        simple_listener.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2.  **Make it Executable**:
    ```bash
    chmod +x ~/ros2_ws/src/my_robot_pkg/my_robot_pkg/listener_node.py
    ```

3.  **Update `setup.py`**: Add the listener node to the `entry_points` in `setup.py`.
    ```python
    entry_points={
        'console_scripts': [
            'talker = my_robot_pkg.talker_node:main',
            'listener = my_robot_pkg.listener_node:main', # Add this line
        ],
    },
    ```

## Building and Running Your Nodes

1.  **Build the Package**: Navigate to your workspace root and build your package.
    ```bash
    cd ~/ros2_ws/
    colcon build --packages-select my_robot_pkg
    ```

2.  **Source the Workspace**: After building, you need to source your workspace to make your new nodes available.
    ```bash
    source install/setup.bash
    ```
    *Note: This must be done in every new terminal where you want to run your nodes.*

3.  **Run the Talker Node**: Open a new terminal and run the talker.
    ```bash
    ros2 run my_robot_pkg talker
    ```
    *Expected Output:*
    ```
    [INFO] [simple_talker]: Publishing: "Hello World: 0"
    [INFO] [simple_talker]: Publishing: "Hello World: 1"
    ...
    ```

4.  **Run the Listener Node**: Open another new terminal and run the listener.
    ```bash
    ros2 run my_robot_pkg listener
    ```
    *Expected Output:*
    ```
    [INFO] [simple_listener]: I heard: "Hello World: 0"
    [INFO] [simple_listener]: I heard: "Hello World: 1"
    ...
    ```

5.  **Visualize the ROS Graph**: While both nodes are running, open a third terminal and run `rqt_graph`.
    ```bash
    rqt_graph
    ```
    You should see a graph showing `simple_talker` and `simple_listener` nodes connected via the `chatter` topic.

## Conclusion

You have successfully created, built, and run your first ROS 2 publisher and subscriber nodes in Python. This establishes the fundamental communication patterns that are central to ROS 2. In the next chapter, we will explore more advanced communication mechanisms like services and actions.
