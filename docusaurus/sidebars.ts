import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  bookSidebar: [
    'intro', // '00-introduction.mdx' slugifies to 'intro'
    {
      type: 'category',
      label: 'Setup and Installation',
      items: [
        'setup/lab-setup',
        'setup/toolchain-install',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-ros2/ros2-architecture',
        'module-ros2/ros2-nodes',
        'module-ros2/topics-services-actions',
        'module-ros2/robot-description-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-digital-twin/gazebo-worlds',
        'module-digital-twin/unity-visualization',
        'module-digital-twin/simulating-sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac Sim)',
      items: [
        'module-ai-robot-brain/introduction',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Systems',
      items: [
        'module-vla-systems/introduction',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Deploying Sim-to-Real Workflows',
      items: [
        'module-sim-to-real/introduction',
      ],
    },
    'capstone-project', // '06-capstone-project.md' slugifies to 'capstone-project'
  ],
};

export default sidebars;
