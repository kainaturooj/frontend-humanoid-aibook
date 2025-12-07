---
sidebar_position: 1
---

# ROS 2 Basics: Nodes, Topics, and Services

## Overview

In this chapter, you'll learn the fundamental concepts of ROS 2 (Robot Operating System 2), which serves as the nervous system for robotic applications. We'll cover the core building blocks: nodes, topics, and services that enable communication between different parts of a robotic system.

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the purpose and function of ROS 2 in robotic systems
- Create and run simple ROS 2 nodes
- Implement publisher-subscriber communication using topics
- Set up client-server communication using services
- Understand the ROS 2 ecosystem and its components

## What is ROS 2?

ROS 2 (Robot Operating System 2) is not an operating system but rather a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

### Key Features of ROS 2

- **Distributed Computing**: ROS 2 allows processes to run on different machines and communicate seamlessly
- **Language Independence**: Supports multiple programming languages (C++, Python, and others)
- **Package Management**: Organizes code into reusable packages
- **Real-time Support**: Better support for real-time systems compared to ROS 1
- **Security**: Built-in security features for safe robot operation

## Core Concepts

### Nodes

A **node** is a process that performs computation. ROS 2 is designed with a distributed architecture where multiple nodes work together to accomplish complex tasks. Each node can perform a specific function, such as sensor processing, motor control, or path planning.

```python
# Example of a simple ROS 2 node in Python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Hello from the minimal node!')

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    minimal_node.get_logger().info('Spinning node...')
    rclpy.spin(minimal_node)

    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics and Message Passing

**Topics** are named buses over which nodes exchange messages. The communication is based on a publish-subscribe pattern where publishers send messages to topics and subscribers receive messages from topics.

- **Publisher**: A node that sends messages to a topic
- **Subscriber**: A node that receives messages from a topic
- **Message**: The data structure exchanged between nodes

### Services

**Services** provide a request-response communication pattern. A client sends a request to a service and waits for a response. This is useful for operations that need to return a result or perform a specific action.

## Practical Exercise: Creating Your First Publisher-Subscriber

Let's create a simple publisher-subscriber example to demonstrate ROS 2 communication.

### Publisher Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
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
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

1. **Basic Publisher-Subscriber**: Create and run the publisher-subscriber example above. Observe how messages are passed between nodes.

2. **Custom Message**: Create a custom message type with multiple fields and use it in your publisher-subscriber example.

3. **Multiple Publishers**: Create multiple publishers publishing to the same topic and observe the behavior.

## Summary

In this chapter, we covered the fundamental concepts of ROS 2:
- Nodes as the basic computational units
- Topics for publish-subscribe communication
- The structure of ROS 2 applications

These concepts form the foundation of ROS 2 communication and will be essential as we move to more advanced topics in subsequent chapters.

## Assessment

1. What is the difference between a publisher and a subscriber in ROS 2?
2. Explain the purpose of a ROS 2 node.
3. How do topics enable communication between nodes?
4. What are the advantages of the publish-subscribe pattern in robotics?