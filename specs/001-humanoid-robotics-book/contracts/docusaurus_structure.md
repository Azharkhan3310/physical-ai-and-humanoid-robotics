# Docusaurus Content Structure Contract

This document defines the file and directory structure for the Docusaurus website that will host the book. Adhering to this structure is critical for a consistent and navigable user experience.

## Root Directory Structure

The project will be a standard Docusaurus website.

```
humanoid-robotics-book/
├── docs/                 # Main book content lives here
├── src/                  # Custom React components and pages
│   ├── components/
│   └── css/
├── static/               # Static assets (images, videos, diagrams)
├── docusaurus.config.js  # Main configuration file
├── package.json
└── sidebars.js           # Sidebar configuration
```

## Book Content (`/docs`)

The `/docs` directory will house all chapters and modules as Markdown (`.md` or `.mdx`) files. The structure will directly mirror the book's outline.

```
docs/
├── 00-introduction.mdx
├── 01-setup/
│   ├── 01-lab-setup.md
│   └── 02-toolchain-install.md
├── 02-module-ros2/
│   ├── 01-ros2-architecture.md
│   ├── 02-ros2-nodes.md
│   ├── 03-topics-services-actions.md
│   └── 04-robot-description-urdf.md
├── 03-module-digital-twin/
│   ├── 01-gazebo-worlds.md
│   ├── 02-unity-visualization.md
│   └── 03-simulating-sensors.md
├── 04-module-isaac/
│   ├── 01-isaac-sim-basics.md
│   ├── 02-perception-isaac-ros.md
│   └── 03-navigation-nav2.md
├── 05-module-vla/
│   ├── 01-voice-control-whisper.md
│   ├── 02-llm-planning.md
│   └── 03-full-vla-pipeline.md
└── 06-capstone-project.md
```

- **Number Prefix**: Files and directories are prefixed with a two-digit number (`01-`, `02-`) to enforce order.
- **File Naming**: Filenames are kebab-case and descriptive.
- **`.mdx`**: Used for files that may require custom React components (e.g., interactive diagrams). Standard `.md` is used otherwise.

## Sidebar (`sidebars.js`)

The `sidebars.js` file will be manually curated to define the navigation structure, ensuring it matches the logical flow of the book.

**Example `sidebars.js` entry:**

```javascript
module.exports = {
  bookSidebar: [
    '00-introduction',
    {
      type: 'category',
      label: 'Setup and Installation',
      items: [
        '01-setup/01-lab-setup',
        '01-setup/02-toolchain-install',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        '02-module-ros2/01-ros2-architecture',
        '02-module-ros2/02-ros2-nodes',
        // ... and so on
      ],
    },
    // ... other modules
    '06-capstone-project',
  ],
};
```

## Static Assets (`/static`)

All images, diagrams, and video files referenced in the book will be stored in the `/static` directory under a structured path.

```
static/
└── img/
    ├── ch02/
    │   ├── ros-graph.png
    │   └── urdf-model.jpg
    └── ch03/
        ├── gazebo-world.png
        └── unity-screenshot.gif
```

- **Referencing in Markdown**: Assets will be referenced using a root path to ensure links do not break.
  - **Example**: `![ROS 2 Graph](/img/ch02/ros-graph.png)`
