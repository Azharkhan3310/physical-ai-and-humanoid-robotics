# Chapter 15: Full VLA Pipeline Integration Example

This directory contains a master ROS 2 launch file (`vla_pipeline.launch.py`) that demonstrates how to bring up the entire Vision-Language-Action pipeline.

## About the Master Launch File

This launch file is the orchestrator for our entire voice control system. It is responsible for launching all the individual nodes we have discussed, ensuring that they can all communicate with each other.

The `vla_pipeline.launch.py` script launches the following components in order:

1.  **Audio Capture Node**: A placeholder for a node that captures audio from a microphone and publishes it. In a real system, you would use a package like [`audio_common`](http://wiki.ros.org/audio_common).
2.  **Whisper Node (from Ch. 13)**: Our speech-to-text engine that converts the raw audio into a text command.
3.  **LLM Planner Node (from Ch. 14)**: Our "brain" that takes the text command and creates a high-level plan.
4.  **Task Action Server (Conceptual)**: A placeholder for the critical node that translates the LLM's plan into low-level robot actions (e.g., calling Nav2). The implementation of this server is the final piece of the puzzle and is left as the main exercise for the capstone project.
5.  **Nav2 Stack (from Ch. 12)**: The complete navigation system, included from our Chapter 12 example, which allows the robot to move autonomously.

## How to Run the Full Pipeline

Running this master launch file requires all prerequisite packages to be built and sourced correctly.

1.  **Prerequisites**:
    -   All custom nodes (`whisper_node`, `llm_planner_node`, `task_action_server`) must be part of a built and sourced ROS 2 package (e.g., `my_voice_control`).
    -   All dependencies for each node (Whisper, Nav2, audio drivers, etc.) must be installed.
    -   An Isaac Sim or Gazebo simulation with the robot must be running.

2.  **Run the Master Launch File**:
    -   From the root of the project, execute the following command:
        ```bash
        ros2 launch examples/ch15/vla_pipeline.launch.py
        ```
    -   This single command will bring up all the nodes. You should see the logs from each node appear in the terminal.

3.  **Operation Flow**:
    -   With everything running, speak a command into the microphone (e.g., "robot, go to the kitchen table").
    -   You can follow the data flow by observing the logs of each node in the terminal:
        1.  The **Whisper Node** will log the transcribed text.
        2.  The **LLM Planner Node** will log the received text and the plan it generates.
        3.  The **Task Action Server** will log the high-level task it receives.
        4.  The **Nav2** logs will show that it has received a goal and is planning a path.
    -   Finally, you should see the robot begin to move in the simulation.
