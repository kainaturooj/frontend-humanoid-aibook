// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  textbookSidebar: [
    {
      type: 'doc',
      id: 'intro',
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Basics - The Robotic Nervous System',
      items: [
        'ros2-basics/ros2-fundamentals',
        'ros2-basics/python-ros2-integration',
        'ros2-basics/urdf-essentials'
      ],
    },
    // Additional modules will be added as they are created
    // {
    //   type: 'category',
    //   label: 'Module 2: Gazebo/Unity - Simulation Environments',
    //   items: [
    //     // Gazebo/Unity content will go here
    //   ],
    // },
    // {
    //   type: 'category',
    //   label: 'Module 3: Isaac - NVIDIA Robotics Platform',
    //   items: [
    //     // Isaac content will go here
    //   ],
    // },
    // {
    //   type: 'category',
    //   label: 'Module 4: VLA - Vision-Language-Action Models',
    //   items: [
    //     // VLA content will go here
    //   ],
    // },
  ],
};

module.exports = sidebars;