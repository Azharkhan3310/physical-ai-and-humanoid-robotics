import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class MinimalFibonacciActionServer(Node):
    def __init__(self):
        super().__init__('minimal_fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Fibonacci Action Server ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                return Fibonacci.Result()
            
            sequence.append(sequence[i] + sequence[i-1])
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            
            # Simulate a long-running process
            self.get_clock().sleep_for_nanoseconds(1_000_000_000) # 1 second

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info('Goal succeeded.')
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = MinimalFibonacciActionServer()
    rclpy.spin(action_server)

if __name__ == '__main__':
    main()
