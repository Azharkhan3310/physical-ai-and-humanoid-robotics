import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Setup and Installation',
      items: [
        'setup/lab-setup',
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
        label: 'Module 2: The Digital Twin',
        items: [
            'module-digital-twin/gazebo-worlds',
            'module-digital-twin/unity-visualization',
            'module-digital-twin/simulating-sensors-and-physics',
        ],
    },
    {
        type: 'category',
        label: 'Module 3: The AI-Robot Brain',
        items: [
            'module-isaac/isaac-sim-basics',
            'module-isaac/perception-isaac-ros',
            'module-isaac/navigation-nav2',
        ],
    },
    {
        type: 'category',
        label: 'Module 4: Vision-Language-Action',
        items: [
            'module-vla/voice-control-whisper',
            'module-vla/llm-planning',
            'module-vla/full-vla-pipeline',
        ],
    },
    'capstone-project',
  ],
};

export default sidebars;
