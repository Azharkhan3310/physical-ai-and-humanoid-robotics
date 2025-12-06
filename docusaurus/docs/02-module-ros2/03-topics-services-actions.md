---
id: topics-services-actions
sidebar_position: 3
title: Topics, Services, and Actions
---

# Topics, Services, and Actions

In Chapter 3, you learned about the core communication mechanisms in ROS 2: Topics, Services, and Actions. In Chapter 4, you built simple publisher/subscriber nodes using Topics. This chapter dives deeper into all three communication patterns, providing practical examples of how to implement and use them to build more complex robot behaviors.

## Topics (Recap and Advanced Usage)

Topics are fundamental for streaming data in a decoupled, many-to-many fashion.

### Quality of Service (QoS) Settings

ROS 2 introduces Quality of Service (QoS) profiles to configure the reliability, durability, and other aspects of topic communication. This is crucial for real-world robotics where network conditions can vary.

*   **Reliability**: `reliable` (guaranteed delivery) vs. `best_effort` (faster, but messages might be lost).
*   **Durability**: `transient_local` (new subscribers receive the last message) vs. `volatile` (only receives messages published after subscription).
*   **History**: `keep_last` (stores a fixed number of messages) vs. `keep_all` (stores all messages).

**Example: Publishing with QoS**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class AdvancedTalker(Node):
    def __init__(self):
        super().__init__('advanced_talker')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(String, 'my_reliable_topic', qos_profile)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Reliable message: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services: Request-Reply Communication

Services are ideal for client-server type interactions where a request is made and a single response is expected.

### Defining a Service Interface

Services require a `.srv` file to define the request and response message types.

*   **`AddTwoInts.srv`**:
    ```
    int64 a
    int64 b
    ---
    int64 sum
    ```
    This defines a service that takes two 64-bit integers (`a` and `b`) and returns their sum (`sum`).

### Implementing a Service Server

A service server node provides the implementation for the service.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Standard example service

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service is ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Implementing a Service Client

A service client node sends a request to the service server.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    
    if len(sys.argv) != 3:
        minimal_client.get_logger().info('Usage: ros2 run <pkg> minimal_client A B')
        minimal_client.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    response = minimal_client.send_request(a, b)
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions: Long-Running Tasks with Feedback

Actions are suitable for tasks that can take a long time to execute and require continuous feedback on their progress, and the ability to be cancelled.

### Defining an Action Interface

Actions require a `.action` file, which is composed of three parts: goal, result, and feedback.

*   **`Fibonacci.action`**:
    ```
    int32 order
    ---
    int32[] sequence
    ---
    int32[] partial_sequence
    ```

### Implementing an Action Server

An action server node processes goals from clients, provides feedback, and sends a final result.

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci # Standard example action

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
```

### Implementing an Action Client

An action client node sends a goal to an action server and handles feedback and results.

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci
import sys

class MinimalFibonacciActionClient(Node):
    def __init__(self):
        super().__init__('minimal_fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        self.get_logger().info(f'Sending goal request: {order}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.partial_sequence}')

def main(args=None):
    rclpy.init(args=args)
    action_client = MinimalFibonacciActionClient()
    
    if len(sys.argv) != 2:
        action_client.get_logger().info('Usage: ros2 run <pkg> minimal_action_client ORDER')
        rclpy.shutdown()
        sys.exit(1)

    order = int(sys.argv[1])
    action_client.send_goal(order)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Conclusion

Topics, Services, and Actions are the backbone of inter-process communication in ROS 2. By mastering these patterns, you can design robust and modular robotic applications. The choice of which pattern to use depends on the specific communication requirements: Topics for streaming data, Services for request-reply, and Actions for long-running, feedback-rich tasks. In the next chapter, we will learn how to describe our robot's physical structure using URDF.
