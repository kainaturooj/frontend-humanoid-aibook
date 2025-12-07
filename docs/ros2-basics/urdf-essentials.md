---
sidebar_position: 3
---

# URDF Essentials for Humanoid Robots

## Overview

Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robot models. This chapter covers the fundamentals of URDF, focusing specifically on its application to humanoid robots. You'll learn how to create robot descriptions that define the physical structure, kinematics, and dynamics of humanoid robots.

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the structure and components of URDF files
- Create basic URDF descriptions for humanoid robots
- Define links, joints, and their properties in URDF
- Visualize and validate URDF models in ROS 2
- Apply URDF best practices for humanoid robotics

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML format that describes robot models in ROS. It defines the physical structure of a robot, including:

- **Links**: Rigid bodies with physical properties (mass, inertia, visual, collision)
- **Joints**: Connections between links with specific degrees of freedom
- **Transmissions**: Mapping between actuators and joints
- **Gazebo plugins**: Simulation-specific properties

## URDF Structure for Humanoid Robots

A humanoid robot URDF typically includes:
- A torso/base link
- Head, arms (with shoulders, elbows, wrists), and legs (with hips, knees, ankles)
- Proper joint definitions for human-like movement

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Torso Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting torso to head -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Key URDF Components for Humanoid Robots

### Links

Links represent rigid bodies in the robot. For humanoid robots, common links include:

- **base_link**: The main body/torso
- **head**: The head with sensors
- **left/right_arm**: Arm segments (shoulder, upper_arm, forearm, hand)
- **left/right_leg**: Leg segments (hip, thigh, shin, foot)

### Joints

Joints define how links connect and move relative to each other. For humanoid robots:

- **Revolute**: Rotational joints (like human joints)
- **Continuous**: Like revolute but unlimited rotation
- **Prismatic**: Linear sliding joints
- **Fixed**: Rigid connections

### Visual and Collision Properties

```xml
<link name="upper_arm">
  <visual>
    <!-- How the link appears visually -->
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </visual>
  <collision>
    <!-- Collision detection geometry -->
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </collision>
  <inertial>
    <!-- Physical properties for simulation -->
    <mass value="0.5"/>
    <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0001"/>
  </inertial>
</link>
```

## Complete Humanoid Robot Example

Here's a more complete example of a simple humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
      <material name="light_blue">
        <color rgba="0.6 0.8 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1.0 0.8 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5.0" velocity="1.0"/>
  </joint>

  <!-- Left shoulder -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Working with URDF in ROS 2

### URDF Validation

Always validate your URDF files:

```bash
# Check URDF syntax
check_urdf /path/to/your/robot.urdf

# Visualize the robot model
ros2 run rviz2 rviz2
```

### Launching with Robot State Publisher

To publish the robot's joint states:

```xml
<launch>
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(find-pkg-share your_robot_description)/urdf/your_robot.urdf"/>
  </node>
</launch>
```

## Exercises

1. **Basic URDF**: Create a simple URDF file for a humanoid robot with torso, head, and two arms.

2. **Joint Limits**: Add proper joint limits to your URDF based on human anatomical constraints.

3. **Visualization**: Load your URDF into RViz2 and verify that it displays correctly.

## Summary

This chapter covered the essentials of URDF for humanoid robots:
- Structure of URDF files with links and joints
- Properties for visual, collision, and inertial modeling
- Examples of humanoid robot descriptions
- Tools for validating and visualizing URDF models

URDF is fundamental to humanoid robotics in ROS 2, enabling simulation, visualization, and control of robot models.

## Assessment

1. What are the main components of a URDF file?
2. Explain the difference between visual, collision, and inertial properties in URDF.
3. What joint types are most appropriate for humanoid robot modeling?
4. How can you validate and visualize a URDF model in ROS 2?