import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String

# Assuming the custom action interface is defined in a package named 'robot_interfaces'
# from robot_interfaces.action import ExecuteTask 

import json
import os

# This is a placeholder for the real action definition.
# In a real project, you would import this from your custom interfaces package.
class ExecuteTask:
    class Goal:
        def __init__(self):
            self.task_name = ""
            self.task_parameters = []

class LLMPlannerNode(Node):
    """
    A ROS 2 node that translates natural language commands into a sequence of
    robot actions by querying a Large Language Model (LLM).
    """
    def __init__(self):
        super().__init__('llm_planner_node')

        # --- Parameters ---
        self.declare_parameter('command_topic', '/robot_command')
        self.declare_parameter('llm_api_key', 'YOUR_API_KEY_HERE') # Default, should be overridden

        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.api_key = self.get_parameter('llm_api_key').get_parameter_value().string_value

        if self.api_key == 'YOUR_API_KEY_HERE' or not self.api_key:
            self.get_logger().warn("LLM API key is not set. Using mock responses.")
            self.use_mock_api = True
        else:
            self.use_mock_api = False
        
        # --- ROS 2 Communication ---
        self.subscription = self.create_subscription(
            String,
            command_topic,
            self.command_callback,
            10
        )
        self.task_action_client = ActionClient(self, ExecuteTask, '/execute_task')

        self.get_logger().info("LLM Planner Node is ready and waiting for commands.")

    def build_prompt(self, command_text):
        """Constructs the full prompt to send to the LLM."""
        return f"""
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
"{command_text}"

## Your Plan (provide as a JSON list of strings):
"""

    def call_llm_api(self, prompt):
        """
        Sends the prompt to the LLM API and returns the response.
        This is a MOCK implementation.
        """
        self.get_logger().info("Sending prompt to LLM API...")
        if self.use_mock_api:
            self.get_logger().warn("Using mock LLM response.")
            # Mock response for a specific command for demonstration purposes
            if "kitchen table" in prompt and "apple" in prompt:
                return '["navigate_to(\'kitchen_table\')", "find_object(\'apple\')", "pick_up_object()"]'
            else:
                return '[]'
        else:
            # In a real implementation, you would use a library like 'openai' or 'requests'
            # to make an HTTP request to the LLM API endpoint.
            #
            # Example using 'openai' library:
            # import openai
            # openai.api_key = self.api_key
            # response = openai.Completion.create(
            #     model="text-davinci-003",
            #     prompt=prompt,
            #     temperature=0.0, # Low temperature for deterministic output
            #     max_tokens=100
            # )
            # return response.choices[0].text
            self.get_logger().error("Real LLM API call is not implemented in this example.")
            return '[]'

    def parse_response(self, llm_response):
        """Parses the JSON string response from the LLM into a Python list."""
        try:
            action_plan = json.loads(llm_response)
            if isinstance(action_plan, list):
                return action_plan
            else:
                return []
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse JSON response from LLM.")
            return []

    async def execute_plan(self, action_plan):
        """Executes the action plan by sending goals to the action server."""
        self.get_logger().info(f"Executing plan: {action_plan}")
        for action_str in action_plan:
            # Parse the action string, e.g., "navigate_to('kitchen_table')"
            try:
                task_name = action_str.split('(')[0]
                # This is a simplified parser for demonstration
                params = action_str.split('(')[1].replace(')', '').replace("'", "").split(',')
                params = [p.strip() for p in params if p.strip()]
            except Exception as e:
                self.get_logger().error(f"Failed to parse action string '{action_str}': {e}")
                continue

            # Send the goal and wait for completion
            goal_msg = ExecuteTask.Goal()
            goal_msg.task_name = task_name
            goal_msg.task_parameters = params
            
            self.get_logger().info(f"Sending goal: {task_name} with params {params}")
            self.task_action_client.wait_for_server()
            
            # In a real node, you would send the goal and handle the async feedback/result
            # future = self.task_action_client.send_goal_async(goal_msg)
            # rclpy.spin_until_future_complete(self, future)
            # ... handle result ...
            self.get_logger().info("Goal sent (in real app, would wait for result).")

        self.get_logger().info("Action plan execution complete.")

    def command_callback(self, msg):
        """Callback for the command subscriber."""
        self.get_logger().info(f"Received command: '{msg.data}'")
        
        prompt = self.build_prompt(msg.data)
        llm_response = self.call_llm_api(prompt)
        action_plan = self.parse_response(llm_response)
        
        if action_plan:
            # Using rclpy.spin_until_future_complete requires careful handling
            # in a real application. For this example, we'll just log.
            self.get_logger().info("Plan generated. In a real app, execution would start.")
            # To run the execution, you would typically do something like:
            # self.create_task(self.execute_plan(action_plan))
        else:
            self.get_logger().warn("No valid action plan generated.")


def main(args=None):
    rclpy.init(args=args)
    try:
        llm_planner_node = LLMPlannerNode()
        rclpy.spin(llm_planner_node)
    except Exception as e:
        rclpy.logging.get_logger("main").error(f"An error occurred in LLM Planner: {e}")
    finally:
        if 'llm_planner_node' in locals() and rclpy.ok():
            llm_planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
