# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `/specs/001-humanoid-robotics-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Test tasks below are for validating code examples and reproducibility, as per the plan.

**Organization**: Tasks are grouped by book module, treating each as a user story for independent implementation.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which book module this task belongs to (e.g., M1 for Module 1, M2 for Module 2)
- Include exact file paths in descriptions as defined in `contracts/docusaurus_structure.md`.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize the Docusaurus book project and overall repository structure.

- [x] T001 Initialize a new Docusaurus website project in a `/docusaurus` sub-directory.
- [x] T002 Configure `docusaurus.config.js` with the book title, theme, and navigation basics.
- [x] T003 Create the directory structure for code examples under an `/examples` root directory.
- [x] T004 [P] Set up a basic `package.json` with linting and formatting scripts (`prettier`, `eslint`).

---

## Phase 2: Foundational Content (Blocking Prerequisites)

**Purpose**: Write the introductory chapters that are essential for all subsequent modules.

- [x] T005 Write Chapter 1: Introduction to Physical AI, in `docusaurus/docs/00-introduction.mdx`.
- [x] T006 Write Chapter 2: Setting Up Your Lab, in `docusaurus/docs/01-setup/01-lab-setup.md`.
- [x] T007 Validate all installation commands from Chapter 2 on a clean Ubuntu 22.04 environment.

---

## Phase 3: Module 1 - The Robotic Nervous System [M1]

**Goal**: Write the chapters and create the examples for the ROS 2 module.
**Independent Test**: A reader can complete this module and have a working ROS 2 simulation of the robot, capable of responding to basic commands.

### Tests for Module 1

- [x] T008 [M1] Create a test script to validate the ROS 2 node graph for the Chapter 4 example.
- [x] T009 [M1] Use `pytest` to write unit tests for the custom ROS 2 service in the Chapter 5 example.

### Implementation for Module 1

- [x] T010 [P] [M1] Write Chapter 3: ROS 2 Architecture, in `docusaurus/docs/02-module-ros2/01-ros2-architecture.md`.
- [x] T011 [P] [M1] Create diagrams for ROS 2 concepts (nodes, topics, etc.) for Chapter 3 and save in `docusaurus/static/img/ch03/`.
- [x] T012 [M1] Write Chapter 4: Creating ROS 2 Nodes (rclpy), in `docusaurus/docs/02-module-ros2/02-ros2-nodes.md`.
- [x] T013 [M1] Develop the example ROS 2 talker/listener nodes for Chapter 4 in `examples/ch04/` and add command outputs/links to the chapter per FR-008.
- [x] T014 [M1] Write Chapter 5: Topics, Services, and Actions, in `docusaurus/docs/02-module-ros2/03-topics-services-actions.md`.
- [x] T015 [M1] Develop the example service and action servers/clients for Chapter 5 in `examples/ch05/` and add command outputs/links to the chapter per FR-008.
- [x] T016 [M1] Write Chapter 6: Describing the Robot (URDF/SDF), in `docusaurus/docs/02-module-ros2/04-robot-description-urdf.md`.
- [x] T017 [M1] Create the base URDF model for the Unitree G1 robot for Chapter 6 in `examples/ch06/` and add command outputs/links to the chapter per FR-008.

---

## Phase 4: Module 2 - The Digital Twin [M2]

**Goal**: Write the chapters and create the examples for simulating the robot in Gazebo and Unity.
**Independent Test**: A reader can load the robot model into a simulated Gazebo world and visualize it in Unity.

- [x] T018 [P] [M2] Write Chapter 7: Building Simulation Worlds (Gazebo), in `docusaurus/docs/03-module-digital-twin/01-gazebo-worlds.md`.
- [x] T019 [M2] Develop the example Gazebo world for Chapter 7 in `examples/ch07/` and add command outputs/links to the chapter per FR-008.
- [x] T020 [P] [M2] Write Chapter 8: Advanced Visualization (Unity), in `docusaurus/docs/03-module-digital-twin/02-unity-visualization.md`.
- [ ] T021 [M2] Configure the Unity project with ROS-TCP-Connector for Chapter 8 in `examples/ch08/` and add command outputs/links to the chapter per FR-008.
- [ ] T022 [P] [M2] Write Chapter 9: Simulating Sensors and Physics, in `docusaurus/docs/03-module-digital-twin/03-simulating-sensors.md`.
- [ ] T023 [M2] Add LiDAR, camera, and IMU sensor plugins to the robot's URDF/SDF model for Chapter 9 in `examples/ch09/` and add command outputs/links to the chapter per FR-008.

---

## Phase 5: Module 3 - The AI-Robot Brain [M3]

**Goal**: Write the chapters and create examples for AI-based perception and navigation using NVIDIA Isaac Sim.
**Independent Test**: A reader can run the Isaac Sim perception pipeline to detect objects and use Nav2 to navigate the robot in a simulated environment.

- [ ] T024 [P] [M3] Write Chapter 10: Isaac Sim for Photorealistic Simulation, in `docusaurus/docs/04-module-isaac/01-isaac-sim-basics.md`.
- [ ] T025 [M3] Create the Isaac Sim environment for Chapter 10 in `examples/ch10/` and add command outputs/links to the chapter per FR-008.
- [ ] T026 [P] [M3] Write Chapter 11: Perception with Isaac ROS, in `docusaurus/docs/04-module-isaac/02-perception-isaac-ros.md`.
- [ ] T027 [M3] Implement an AprilTag detection pipeline using Isaac ROS GEMs for Chapter 11 in `examples/ch11/` and add command outputs/links to the chapter per FR-008.
- [ ] T028 [P] [M3] Write Chapter 12: Navigation with Nav2, in `docusaurus/docs/04-module-isaac/03-navigation-nav2.md`.
- [ ] T029 [M3] Configure Nav2 for the simulated robot and test navigation in the Isaac Sim environment for Chapter 12 in `examples/ch12/` and add command outputs/links to the chapter per FR-008.

---

## Phase 6: Module 4 - Vision-Language-Action [M4]

**Goal**: Write the chapters and create examples for the complete VLA pipeline.
**Independent Test**: A reader can give a voice command to the robot in simulation and have it execute a planned task.

- [ ] T030 [P] [M4] Write Chapter 13: Voice Control with Whisper, in `docusaurus/docs/05-module-vla/01-voice-control-whisper.md`.
- [ ] T031 [M4] Implement the Whisper-to-text ROS 2 node for Chapter 13 in `examples/ch13/` and add command outputs/links to the chapter per FR-008.
- [ ] T032 [P] [M4] Write Chapter 14: LLM-based Planning, in `docusaurus/docs/05-module-vla/02-llm-planning.md`.
- [ ] T033 [M4] Implement the LLM planner ROS 2 node that converts text to action goals for Chapter 14 in `examples/ch14/` and add command outputs/links to the chapter per FR-008.
- [ ] T034 [P] [M4] Write Chapter 15: The Full VLA Pipeline, in `docusaurus/docs/05-module-vla/03-full-vla-pipeline.md`.
- [ ] T035 [M4] Integrate the Whisper, LLM, and Nav2 nodes into a complete pipeline for Chapter 15 in `examples/ch15/` and add command outputs/links to the chapter per FR-008.

---

## Phase 7: Capstone Project [C1]

**Goal**: Write the final capstone project chapter, integrating all previous modules.
**Independent Test**: The full project is reproducible and achieves the primary success criteria of the book.

- [ ] T036 [C1] Write Chapter 16: Capstone Project - Autonomous Humanoid Assistant, in `docusaurus/docs/06-capstone-project.md`.
- [ ] T037 [C1] Assemble and document the final integrated code for the capstone project in `examples/ch16/` and add command outputs/links to the chapter per FR-008.
- [ ] T038 [C1] Create a comprehensive `README.md` for the final project explaining how to run it.
- [ ] T039 [C1] Record a video demonstration of the final project running in simulation.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final review, cleanup, and deployment.

- [ ] T040 [P] Perform a full editorial review of all chapters for clarity, grammar, and style.
- [ ] T041 [P] Review all chapters and add diagrams where needed to improve clarity, per FR-006.
- [ ] T042 [P] Validate all code examples and commands one final time against the target hardware.
- [ ] T043 [P] Check all internal and external links in the Docusaurus site.
- [ ] T044 Update `sidebars.js` to correctly reflect the final chapter structure.
- [ ] T045 Build the Docusaurus site for production (`npm run build`).
- [ ] T046 Configure GitHub Actions to deploy the Docusaurus site to GitHub Pages.
- [ ] T047 Run a final validation of the live GitHub Pages site.