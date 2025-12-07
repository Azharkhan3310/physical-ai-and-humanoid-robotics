# Chapter 14: LLM-based Planner ROS 2 Node

This directory contains an example ROS 2 node (`llm_planner_node.py`) that acts as a high-level task planner by converting natural language commands into a sequence of robot actions using a Large Language Model (LLM).

## About the Node

The `llm_planner_node.py` script demonstrates the core logic of a VLA (Vision-Language-Action) "brain".

1.  **Subscribes to Commands**: It listens for text commands on the `/robot_command` topic, which is published to by our Whisper node.
2.  **Builds a Prompt**: When it receives a command, it constructs a detailed "prompt" that provides the LLM with context about the robot's capabilities and what it needs to do.
3.  **Calls an LLM API (Mocked)**: It sends this prompt to an LLM. **Note**: This example includes a *mock* API call. It does not actually connect to a live LLM service. Instead, it returns a pre-defined plan for a specific command to demonstrate the logic.
4.  **Parses the Response**: It parses the JSON response from the LLM to extract a list of actions.
5.  **Sends Goals to an Action Server**: It iterates through the action plan and (conceptually) sends each action as a goal to an `/execute_task` action server. The waiting and execution logic is simplified for clarity.

## Prerequisites

1.  **ROS 2 Action Interface**: This node assumes the existence of a custom action named `ExecuteTask` in a package called `robot_interfaces`. You would need to define this action (`.action` file) and build the package. The node file contains a placeholder class for this so it can run for demonstration.
2.  **An Action Server**: To fully execute the plan, you would need a separate ROS 2 node that implements an action server for the `/execute_task` action. This server would be responsible for calling Nav2, the manipulation stack, etc., based on the received task.
3.  **(Optional) LLM API Key**: To connect to a real LLM, you would need to install the appropriate Python library (e.g., `openai`) and provide an API key. You can pass the API key as a ROS 2 parameter when running the node.

## How to Run This Node

1.  **Place in a ROS 2 Package**:
    Just like the Whisper node, this script should be part of a ROS 2 Python package to be executable. Follow the same steps as in Chapter 13 to add it to a package (e.g., `my_voice_control`) and expose it as a console script in `setup.py`.

2.  **Launch the System**:
    -   Run the Whisper node from Chapter 13 so that it's publishing to `/robot_command`.
    -   (Optional) Run your `ExecuteTask` action server.
    -   Run the planner node:
        ```bash
        # Run with the mock API
        ros2 run my_voice_control llm_planner_node
        
        # To run with a real API key (once implemented)
        ros2 run my_voice_control llm_planner_node --ros-args -p llm_api_key:="sk-..."
        ```

3.  **Verify the Operation**:
    -   Speak a command into the microphone. For the mock to work, say **"go to the kitchen table and get the apple"**.
    -   **Observe the Whisper Node's Logs**: You should see it transcribe your speech.
    -   **Observe the Planner Node's Logs**:
        -   It will log that it received the command.
        -   It will log that it is using the mock API.
        -   It will log the plan it "generated": `Executing plan: ["navigate_to('kitchen_table')", "find_object('apple')", "pick_up_object()"]`.
        -   It will then log each goal as it "sends" it.
