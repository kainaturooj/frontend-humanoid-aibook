---
sidebar_position: 2
---

# Python Agents → ROS 2 Control (rclpy)

## Overview

This chapter focuses on integrating Python applications with ROS 2 using the `rclpy` library. You'll learn how to create Python-based agents that can control robotic systems through ROS 2, enabling the development of sophisticated robotic behaviors using Python's rich ecosystem.

## Learning Objectives

By the end of this chapter, you will be able to:
- Use the rclpy library to create Python nodes in ROS 2
- Implement complex robot control algorithms in Python
- Integrate Python machine learning libraries with ROS 2
- Design Python-based agents for robotic task execution
- Handle ROS 2 messages and services from Python

## Introduction to rclpy

`rclpy` is the Python client library for ROS 2. It provides a Python API to interact with the ROS 2 system, allowing you to create nodes, publish and subscribe to topics, provide and use services, and more.

### Installing rclpy

rclpy is typically installed as part of a ROS 2 distribution. If you're setting up a standalone environment, you can install it using:

```bash
pip install rclpy
```

## Creating Python Nodes with rclpy

Let's explore how to create different types of nodes using rclpy:

### Simple Publisher Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create publishers for different robot components
        self.motor_pub = self.create_publisher(String, 'motor_commands', 10)
        self.sensor_sub = self.create_subscription(
            String, 'sensor_data', self.sensor_callback, 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Robot Controller initialized')

    def sensor_callback(self, msg):
        # Process sensor data
        sensor_data = json.loads(msg.data)
        self.get_logger().info(f'Received sensor data: {sensor_data}')

    def control_loop(self):
        # Implement control logic here
        command = {
            'motor_id': 'left_wheel',
            'velocity': 1.0,
            'timestamp': self.get_clock().now().nanoseconds
        }

        msg = String()
        msg.data = json.dumps(command)
        self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Python Agent Patterns

### State Machine Agent

```python
from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    SENSING = "sensing"
    ERROR = "error"

class StateMachineAgent(Node):
    def __init__(self):
        super().__init__('state_machine_agent')

        self.current_state = RobotState.IDLE
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)
        self.command_subscriber = self.create_subscription(
            String, 'commands', self.command_callback, 10)

        self.state_timer = self.create_timer(0.5, self.state_machine)

        self.get_logger().info(f'State machine initialized in {self.current_state.value} state')

    def command_callback(self, msg):
        command = msg.data.lower()

        if command == 'start' and self.current_state == RobotState.IDLE:
            self.current_state = RobotState.MOVING
        elif command == 'sense' and self.current_state in [RobotState.IDLE, RobotState.MOVING]:
            self.current_state = RobotState.SENSING
        elif command == 'stop':
            self.current_state = RobotState.IDLE

    def state_machine(self):
        # Execute behavior based on current state
        if self.current_state == RobotState.MOVING:
            self.execute_movement()
        elif self.current_state == RobotState.SENSING:
            self.execute_sensing()
        elif self.current_state == RobotState.ERROR:
            self.handle_error()

        # Publish current state
        state_msg = String()
        state_msg.data = self.current_state.value
        self.state_publisher.publish(state_msg)

    def execute_movement(self):
        self.get_logger().info('Executing movement behavior')

    def execute_sensing(self):
        self.get_logger().info('Executing sensing behavior')

    def handle_error(self):
        self.get_logger().info('Handling error state')

def main(args=None):
    rclpy.init(args=args)
    agent = StateMachineAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Machine Learning Libraries

Python's rich ML ecosystem can be integrated with ROS 2 for intelligent robot behaviors:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
# Example integration with popular ML libraries
# import tensorflow as tf
# import torch
# import sklearn

class MLCtrlAgent(Node):
    def __init__(self):
        super().__init__('ml_control_agent')

        # Subscribe to sensor data
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

        # Publisher for control commands
        self.ctrl_pub = self.create_publisher(String, 'control_commands', 10)

        # Initialize ML model (example placeholder)
        self.ml_model = self.initialize_model()

        self.get_logger().info('ML Control Agent initialized')

    def initialize_model(self):
        # Initialize your ML model here
        # This could be a pre-trained model loaded from file
        self.get_logger().info('ML model initialized')
        return None  # Placeholder

    def image_callback(self, msg):
        # Process image and run inference
        image_data = self.process_image(msg)
        control_action = self.run_inference(image_data)

        # Publish control command
        ctrl_msg = String()
        ctrl_msg.data = control_action
        self.ctrl_pub.publish(ctrl_msg)

    def process_image(self, img_msg):
        # Convert ROS image message to format suitable for ML model
        # Implementation depends on your specific use case
        return np.zeros((1,))  # Placeholder

    def run_inference(self, data):
        # Run ML model inference
        # This is where your intelligent behavior happens
        return "move_forward"  # Placeholder action

def main(args=None):
    rclpy.init(args=args)
    agent = MLCtrlAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Python-ROS 2 Integration

1. **Error Handling**: Always implement proper error handling in your Python ROS 2 nodes
2. **Resource Management**: Properly clean up resources when nodes are destroyed
3. **Threading**: Be aware of threading implications when using Python libraries with ROS 2
4. **Performance**: Consider performance implications of Python vs. C++ for time-critical applications

## Exercises

1. **Simple Controller**: Create a Python node that subscribes to sensor data and publishes motor commands based on a simple control law.

2. **Behavior Tree Agent**: Implement a more complex agent using behavior trees or decision trees in Python.

3. **ML Integration**: Integrate a simple machine learning model (like scikit-learn) with ROS 2 to create an intelligent agent.

## Summary

This chapter covered the integration of Python with ROS 2:
- Using rclpy to create Python-based ROS 2 nodes
- Implementing various agent patterns (state machines, controllers)
- Integrating machine learning libraries with ROS 2
- Best practices for Python-ROS 2 development

Python's flexibility combined with ROS 2's communication framework enables the development of sophisticated robotic agents capable of complex behaviors.

## Assessment

1. What is rclpy and how does it enable Python-ROS 2 integration?
2. Describe the differences between a simple publisher and a state machine agent.
3. How can machine learning libraries be integrated with ROS 2 using Python?
4. What are the key considerations when developing Python agents for robotics?