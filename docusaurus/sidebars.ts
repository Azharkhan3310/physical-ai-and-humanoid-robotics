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
    '00-introduction',
    {
      type: 'category',
      label: 'Setup and Installation',
      items: [
        '01-setup/01-lab-setup',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        '02-module-ros2/01-ros2-architecture',
        '02-module-ros2/02-ros2-nodes',
        '02-module-ros2/03-topics-services-actions',
        '02-module-ros2/04-robot-description-urdf',
      ],
    },
    // Add other modules here as they are written
  ],
};

export default sidebars;
