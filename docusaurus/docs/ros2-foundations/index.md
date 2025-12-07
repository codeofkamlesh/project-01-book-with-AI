---
sidebar_position: 2
---

# ROS2 Foundations

## Overview

This chapter introduces the fundamental concepts of Robot Operating System 2 (ROS 2), the middleware framework that enables communication between different components of a robotic system. ROS 2 provides the infrastructure for building distributed robotic applications with support for multiple programming languages, real-time systems, and distributed computing.

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the core concepts of ROS 2 architecture
- Create and run basic ROS 2 nodes
- Understand topics, services, and actions
- Work with parameters and launch files
- Implement basic robot control patterns

## Table of Contents

1. [Introduction to ROS 2](#introduction-to-ros-2)
2. [Nodes and Communication](#nodes-and-communication)
3. [Topics, Services, and Actions](#topics-services-and-actions)
4. [Parameters and Launch Files](#parameters-and-launch-files)
5. [Basic Robot Control](#basic-robot-control)
6. [Hands-on Examples](#hands-on-examples)

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is not an operating system but rather a collection of software frameworks and tools that help developers create robotic applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

### Key Features of ROS 2

- **Distributed Computing**: ROS 2 allows nodes to run on different machines and communicate over a network
- **Language Independence**: Support for multiple programming languages including Python, C++, and others
- **Real-time Support**: Enhanced real-time capabilities compared to ROS 1
- **Security**: Built-in security features for protecting robotic systems
- **Middleware Agnostic**: Uses DDS (Data Distribution Service) for communication, making it middleware-agnostic

### Architecture Components

ROS 2 architecture consists of several layers:

1. **Application Layer**: Your robotic applications and algorithms
2. **Client Library Layer**: ROS 2 client libraries (rclcpp, rclpy, etc.)
3. **ROS Middleware (RMW)**: Interface between client libraries and DDS implementations
4. **DDS/RTPS**: Communication layer implementing the DDS specification
5. **OS/Platform**: Operating system and hardware platform

## Nodes and Communication

### What is a Node?

A node is a process that performs computation. In ROS 2, nodes are designed to be composable and distributed. Each node typically performs a specific task and communicates with other nodes through topics, services, or actions.

### Creating a Simple Node

Here's a basic ROS 2 node in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal node!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle

ROS 2 nodes have a well-defined lifecycle that includes states like unconfigured, inactive, active, and finalized. This allows for better resource management and system control.

## Topics, Services, and Actions

### Topics

Topics enable asynchronous, many-to-many communication between nodes using a publish-subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics.

```python
# Publisher example
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
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### Services

Services provide synchronous, request-response communication. A service client sends a request and waits for a response from a service server.

### Actions

Actions are for long-running tasks that may provide feedback during execution and can be canceled. They're ideal for tasks like navigation or manipulation.

## Parameters and Launch Files

### Parameters

Parameters in ROS 2 allow you to configure nodes at runtime. They can be set at launch time or changed dynamically.

```python
# Parameter example
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_param', 'default_value')

        # Get parameter value
        param_value = self.get_parameter('my_param').value
        self.get_logger().info(f'Parameter value: {param_value}')
```

### Launch Files

Launch files allow you to start multiple nodes with specific configurations simultaneously.

```python
# launch/example_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='talker',
            parameters=[
                {'param_name': 'param_value'}
            ]
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener'
        )
    ])
```

## Basic Robot Control

### Robot Control Patterns

ROS 2 provides several patterns for controlling robots:

1. **Joint Trajectory Control**: For precise joint movements
2. **Velocity Control**: For continuous motion control
3. **Position Control**: For reaching specific positions
4. **Impedance Control**: For compliant interaction with the environment

### TF (Transforms) Framework

The TF framework is crucial for robot control, allowing you to keep track of coordinate frames over time.

## Hands-on Examples

### Example 1: Publisher-Subscriber

Create a publisher that sends messages and a subscriber that receives and processes them.

### Example 2: Service Server-Client

Create a service that performs a calculation and a client that requests the calculation.

### Example 3: Parameter Configuration

Create a node that uses parameters for configuration and demonstrate changing parameters at runtime.

## Summary

This chapter covered the foundational concepts of ROS 2 that will be essential for the rest of this book. Understanding nodes, topics, services, actions, parameters, and launch files is crucial for building robotic applications. In the next chapter, we'll explore how to use these concepts in simulation environments.

## References and Further Reading

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Design ros2](https://design.ros2.org/)