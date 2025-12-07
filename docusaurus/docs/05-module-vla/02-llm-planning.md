---
id: llm-planning
sidebar_position: 2
title: LLM-based Planning
---

# Chapter 14: LLM-based Planning

We have successfully converted spoken words into text. Now, we need to understand the *intent* behind that text and convert it into a structured plan that our robot can execute. This is where the power of Large Language Models (LLMs) comes in.

This chapter explores how to use an LLM, such as one from the GPT family or an open-source alternative, to function as the "brain" of our robot, translating natural language commands into a sequence of actionable tasks.

## The Role of the LLM in Robotics

An LLM's strength lies in its ability to understand context, nuance, and ambiguity in human language. In our VLA pipeline, the LLM acts as a high-level task planner.

**Input**: A natural language command (e.g., "bring me the red block from the other table").
**Output**: A structured, sequential list of robot actions (e.g., `["navigate_to('other_table')", "find_object('red_block')", "pick_up_object()", "navigate_to('home')"]`).

The LLM is not responsible for *how* the robot executes these actions (that's the job of Nav2 and the manipulation stack), but rather for determining *what* actions to take and in what order.

## Prompt Engineering for Robotics

The key to getting the desired output from an LLM is **prompt engineering**. The prompt is the context and question we provide to the model. For a robotics task planner, a good prompt must include:

1.  **The Robot's Capabilities**: A clear, concise description of the actions the robot can perform. This acts as the LLM's "API documentation" for the robot.
2.  **The Goal**: The transcribed voice command from the user.
3.  **The Output Format**: Strict instructions on how the LLM should format its response (e.g., as a JSON array of strings).
4.  **Constraints and Rules**: Safety information, rules about what the robot is allowed or not allowed to do, and how to handle ambiguity.

### Example Prompt

Here is an example of what a well-structured prompt might look like:

```text
You are an expert robot task planner. Your job is to convert a user's command into a sequence of simple, executable actions.

## Available Robot Actions:
- navigate_to(location): Moves the robot to a named location.
- find_object(object_description): Looks for an object matching the description.
- pick_up_object(): Picks up the object that has been found.
- place_object(): Places the currently held object.

## Known Locations:
- 'home_base'
- 'charging_dock'
- 'kitchen_table'

## Rules:
- You can only use the actions listed above.
- If a command is unclear or requires an action you don't have, respond with an empty list.
- Break down complex commands into a logical sequence of actions.

## User Command:
"go to the kitchen table and get the apple"

## Your Plan (provide as a JSON list of strings):
```

Given this prompt, a capable LLM would ideally respond with:
```json
[
  "navigate_to('kitchen_table')",
  "find_object('apple')",
  "pick_up_object()"
]
```

## Architecture of an LLM Planner Node

A ROS 2 node that implements this logic would have the following structure:

1.  **Command Subscriber**: Subscribes to the `/robot_command` topic (from our Whisper node) to receive transcribed text.
2.  **LLM API Client**: When a command is received, the node constructs the prompt and sends it to an LLM API (e.g., OpenAI's API or a locally hosted LLM).
3.  **Response Parser**: It parses the LLM's response, validates that it's in the correct JSON format, and extracts the list of actions.
4.  **Action Client**: For each action in the sequence, the node uses a ROS 2 Action Client to send a goal to the appropriate action server (e.g., an `/execute_task` server as defined in our `ros_interfaces.md` contract). It waits for one action to complete before sending the next.

```python
# High-level pseudocode for the LLM planner node

class LLMPlannerNode(Node):
    def __init__(self):
        # ... setup subscribers, publishers, action clients ...
        self.create_subscription(String, '/robot_command', self.command_callback, 10)
        self.task_action_client = ActionClient(self, ExecuteTask, '/execute_task')

    def command_callback(self, msg):
        command_text = msg.data
        
        # 1. Construct the prompt
        prompt = self.build_prompt(command_text)
        
        # 2. Send to LLM and get response
        llm_response = self.call_llm_api(prompt)
        
        # 3. Parse the plan
        action_plan = self.parse_response(llm_response)
        
        # 4. Execute the plan sequentially
        for action in action_plan:
            goal = self.create_goal_from_action(action)
            self.task_action_client.send_goal_async(goal)
            # ... wait for result before sending the next goal ...

# ... rest of the node ...
```

## Example: LLM Planner Node

A complete ROS 2 node that implements the LLM planning logic described above is available in the examples for this chapter. The node includes a mock LLM API call, so it can be run without needing a real API key.

The README file includes full details on the node's architecture, prerequisites, and how to run it to see the full (mock) pipeline in action.

-   [**Chapter 14 Example: README**](../../../examples/ch14/README.md)
-   [**Chapter 14 Example: Python Node**](../../../examples/ch14/llm_planner_node.py)

## Next Steps

This chapter has outlined the "brain" of our operation. We now have a conceptual framework for converting natural language into robot actions. The next steps are to implement this planner node and the underlying action server that will connect our high-level plan to the robot's low-level capabilities like Nav2.
