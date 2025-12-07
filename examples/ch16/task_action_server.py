import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor

# Placeholder for the custom action interface from robot_interfaces
class ExecuteTask:
    class Action:
        Goal = type('Goal', (), {'task_name': '', 'task_parameters': []})
        Result = type('Result', (), {'success': False, 'message': ''})
        Feedback = type('Feedback', (), {'status': ''})

# Placeholder for the Nav2 navigation action interface
class NavigateToPose:
    class Action:
        Goal = type('Goal', (), {})

class TaskActionServer(Node):
    """
    This server acts as a bridge between high-level tasks from the LLM
    and low-level ROS 2 action servers like Nav2.
    """
    def __init__(self):
        super().__init__('task_action_server')
        
        self.get_logger().info("Initializing Task Action Server...")
        
        # The Action Server that the LLM Planner calls
        self._action_server = ActionServer(
            self,
            ExecuteTask.Action,
            '/execute_task',
            self.execute_callback
        )
        
        # An Action Client to call the Nav2 stack
        self._nav_client = ActionClient(self, NavigateToPose.Action, 'navigate_to_pose')

        self.get_logger().info("Task Action Server is ready.")

    def execute_callback(self, goal_handle):
        """
        This callback is triggered when the LLM Planner sends a goal.
        It dispatches the task to the appropriate handler.
        """
        task_name = goal_handle.request.task_name
        task_params = goal_handle.request.task_parameters

        self.get_logger().info(f"Received task: '{task_name}' with params: {task_params}")

        feedback_msg = ExecuteTask.Action.Feedback()
        
        if task_name == 'navigate_to':
            feedback_msg.status = f"Executing navigation to {task_params[0]}"
            goal_handle.publish_feedback(feedback_msg)
            
            # --- This is where you would call the Nav2 action server ---
            self.get_logger().info("Calling Nav2 action server (MOCK)...")
            # nav_goal = NavigateToPose.Action.Goal()
            # ... (set nav_goal properties based on a lookup of the location)
            # self._nav_client.wait_for_server()
            # future = self._nav_client.send_goal_async(nav_goal)
            # rclpy.spin_until_future_complete(self, future)
            # ... handle result of navigation ...
            
            # For this example, we'll just simulate success after a delay
            import time
            time.sleep(5) 
            self.get_logger().info("Navigation succeeded (MOCK).")
            
            goal_handle.succeed()
            result = ExecuteTask.Action.Result()
            result.success = True
            result.message = "Navigation complete."
            return result
            
        elif task_name == 'find_object':
            feedback_msg.status = f"Searching for {task_params[0]}"
            goal_handle.publish_feedback(feedback_msg)
            
            # --- This would call a perception action server ---
            self.get_logger().info("Calling perception system (MOCK)...")
            time.sleep(3)
            self.get_logger().info("Object found (MOCK).")

            goal_handle.succeed()
            result = ExecuteTask.Action.Result()
            result.success = True
            result.message = "Object found."
            return result
            
        else:
            self.get_logger().error(f"Unknown task: {task_name}")
            goal_handle.abort()
            result = ExecuteTask.Action.Result()
            result.success = False
            result.message = f"Task '{task_name}' is not implemented."
            return result

def main(args=None):
    rclpy.init(args=args)
    try:
        task_action_server = TaskActionServer()
        # Use a MultiThreadedExecutor to handle multiple callbacks concurrently
        executor = MultiThreadedExecutor()
        rclpy.spin(task_action_server, executor=executor)
    except Exception as e:
        rclpy.logging.get_logger("main").error(f"An error occurred in Task Action Server: {e}")
    finally:
        if 'task_action_server' in locals() and rclpy.ok():
            task_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
