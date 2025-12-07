# Chapter 16: Capstone Project - Autonomous Humanoid Assistant

Welcome to the complete code for the Capstone Project. This directory contains the final integrated nodes and launch file to run the full Vision-Language-Action (VLA) pipeline.

## Architecture Overview

This project integrates several ROS 2 nodes and systems to achieve voice-controlled autonomy:

1.  **Audio Input (`audio_common` package)**: Captures audio from a microphone.
2.  **Whisper Node (`whisper_node.py`)**: Transcribes the audio into a text command.
3.  **LLM Planner Node (`llm_planner_node.py`)**: Takes the text command and generates a high-level task plan (e.g., `["navigate_to('kitchen')", "find_object('soda')"]`).
4.  **Task Action Server (`task_action_server.py`)**: Receives tasks from the LLM planner and calls the appropriate low-level ROS 2 systems to execute them. **This is a mock implementation** that simulates success for demonstration.
5.  **Nav2 Stack**: The full navigation system that executes the `navigate_to` tasks.

## Prerequisites

This project assumes you have a complete ROS 2 Humble environment set up as described in the book's early chapters, including:
-   ROS 2 Humble Desktop
-   Gazebo or Isaac Sim installed and configured
-   A working ROS 2 workspace (e.g., `~/ros2_ws`)
-   All Python dependencies for the Whisper and LLM nodes installed (`openai-whisper`, `torch`, etc.)
-   A configured microphone and the `audio_common` ROS 2 package.

## Step 1: Create the ROS 2 Package

All the Python nodes (`whisper_node.py`, `llm_planner_node.py`, `task_action_server.py`) need to reside in a ROS 2 package to be executable.

1.  **Navigate to your workspace's `src` directory**:
    ```bash
    cd ~/ros2_ws/src
    ```
2.  **Create a new package**:
    ```bash
    ros2 pkg create --build-type ament_python --license Apache-2.0 my_voice_control
    ```
3.  **Copy the nodes**:
    -   Copy `whisper_node.py` (from `examples/ch13`), `llm_planner_node.py` (from `examples/ch14`), and `task_action_server.py` (from this directory) into the `my_voice_control/my_voice_control` directory.
    -   Copy `capstone.launch.py` (from this directory) into a new `launch` directory: `my_voice_control/launch`.

4.  **Edit `setup.py`**:
    Open `my_voice_control/setup.py` and add the entry points for the nodes and the launch file directory.
    ```python
    # setup.py
    # ...
    setup(
        # ...
        data_files=[
            # ... other files
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ],
        entry_points={
            'console_scripts': [
                'whisper_node = my_voice_control.whisper_node:main',
                'llm_planner_node = my_voice_control.llm_planner_node:main',
                'task_action_server = my_voice_control.task_action_server:main',
            ],
        },
    )
    ```

5.  **Build the Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_voice_control
    ```

## Step 2: Run the Full System

1.  **Start your Simulation**: Launch Isaac Sim or Gazebo and load the robot model in its environment.

2.  **Source your workspace**:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

3.  **Run the Master Launch File**:
    ```bash
    ros2 launch my_voice_control capstone.launch.py
    ```
    This will start all the nodes, including Nav2. Wait for the logs to indicate that all servers are ready.

## Step 3: Give a Command

1.  **Speak into the microphone**:
    -   "Robot, go to the kitchen table and find the apple."

2.  **Observe the Pipeline**:
    -   Watch the terminals for the log outputs from each node.
    -   You should see the text transcribed, the plan generated, and the mock Task Action Server "executing" the navigation and find tasks.
    -   In your simulation, the robot should begin moving towards the kitchen table, controlled by Nav2.

This completes the full Vision-Language-Action pipeline and the final project of this book.
