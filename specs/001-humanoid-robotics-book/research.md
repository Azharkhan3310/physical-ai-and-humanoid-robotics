# Research & Decisions: Physical AI & Humanoid Robotics Book

This document records the key technology and pedagogical decisions for the book, including rationales and alternatives considered.

---

### Decision: ROS 2 Humble as the primary ROS 2 distribution

- **Decision**: The book will standardize on ROS 2 Humble Hawksbill.
- **Rationale**: Humble is a Long-Term Support (LTS) release with support until May 2027. This provides a stable, reliable, and well-supported platform for a book, ensuring that examples and instructions remain valid for years. It has a mature ecosystem of packages and excellent documentation.
- **Alternatives Considered**:
  - **ROS 2 Iron**: A more recent release with newer features, but it is a non-LTS release with a much shorter support window. For a book that needs to remain relevant, stability is more important than cutting-edge features.
  - **ROS 1 Noetic**: The final ROS 1 release. While still widely used, it is legacy technology. This book is focused on teaching modern robotics practices, making ROS 2 the only viable choice.

---

### Decision: Use both Gazebo and Unity for simulation

- **Decision**: The book will teach Gazebo for core physics simulation and ROS 2 integration, and introduce Unity for high-fidelity visualization and specific advanced scenarios.
- **Rationale**: This combination provides the best of both worlds. Gazebo offers tight integration with the ROS ecosystem and is excellent for simulating physics and sensors. Unity, via the ROS-TCP-Connector, provides superior graphics, making it ideal for creating visually impressive simulations and for projects where high-fidelity rendering is important (e.g., training a vision-based ML model).
- **Alternatives Considered**:
  - **Gazebo only**: Sufficient for most ROS 2 work but lacks the graphical fidelity of a modern game engine.
  - **Unity only**: Possible, but the ROS 2 integration is less mature than Gazebo's, and Gazebo is more of a standard in the ROS community.

---

### Decision: Use NVIDIA Isaac Sim as the primary platform for AI and perception

- **Decision**: NVIDIA Isaac Sim will be the main tool for teaching AI-driven perception, navigation, and synthetic data generation.
- **Rationale**: Isaac Sim is built on NVIDIA Omniverse and provides photorealistic simulation, which is crucial for modern AI and sim-to-real workflows. Its tight integration with Isaac ROS GEMs (GPU-accelerated packages for perception, VSLAM, etc.) and Nav2 makes it the most powerful and relevant platform for teaching AI in a robotics context. The ability to generate synthetic datasets is a key skill for modern roboticists.
- **Alternatives Considered**:
  - **Webots**: A strong, open-source simulator. However, it lacks the deep integration with the NVIDIA AI stack and the photorealistic rendering capabilities of Isaac Sim.
  - **PyBullet**: A lightweight and fast physics simulator, excellent for reinforcement learning research. However, it is less focused on holistic robot simulation (sensors, environments) and lacks the visual fidelity of Isaac Sim.

---

### Decision: Target NVIDIA Jetson Orin (Nano/NX) for edge deployment

- **Decision**: The book will use the NVIDIA Jetson Orin Nano for entry-level "sim-to-real" examples and the Orin NX for more performance-intensive VLA tasks.
- **Rationale**: The Jetson Orin series offers the best-in-class performance-per-watt for edge AI applications. The shared CUDA architecture with NVIDIA workstations simplifies the sim-to-real transition. The Nano provides an accessible entry point, while the NX offers a clear upgrade path for more complex models. The entire NVIDIA Jetson ecosystem (JetPack SDK, etc.) is well-documented and supported.
- **Alternatives Considered**:
  - **Raspberry Pi 5**: A popular and affordable SBC, but it lacks the GPU acceleration required to run the AI/perception models taught in the book.
  - **Laptop/Workstation only**: This would prevent the book from teaching the critical "sim-to-real" step of deploying to a low-power edge device.

---

### Decision: Recommend the ReSpeaker Microphone Array for voice commands

- **Decision**: The ReSpeaker 2-Mic/4-Mic Array will be the recommended hardware for capturing voice commands for the Whisper-based VLA module.
- **Rationale**: The ReSpeaker series offers a good balance of performance and affordability. They have open-source, well-maintained drivers for Linux, perform well for far-field voice capture, and are readily available. This makes them a reliable choice for a book project.
- **Alternatives Considered**:
  - **Standard USB microphone**: Simpler, but may not have the quality needed for reliable speech recognition with Whisper, especially in noisy environments.
  - **Built-in laptop microphone**: Quality varies too widely and is not a reproducible hardware target.

---

### Decision: Use the Unitree G1/G2 as the reference humanoid robot model

- **Decision**: The book will use a URDF/SDF model based on the Unitree G1/G2 humanoid robot for all simulation and examples.
- **Rationale**: The Unitree G1/G2 represents the state-of-the-art in commercially available, developer-focused humanoid robots. Using a model based on a real, cutting-edge robot makes the project more exciting and relevant. While users are not expected to own one, the simulation provides a realistic platform for learning.
- **Alternatives Considered**:
  - **Boston Dynamics Atlas**: An iconic robot, but it is a research platform, not a commercially available product.
  - **Generic humanoid model**: Less inspiring and may not accurately represent the dynamics of a real-world system.

---

### Decision: Focus on an on-premise, local development lab

- **Decision**: The book's curriculum will be based entirely on a local development setup (the "Digital Twin Workstation" and "Edge AI Kit").
- **Rationale**: Robotics development, especially involving simulation and hardware, requires low-latency interaction and high-bandwidth data transfer, which is best achieved on a local machine. An on-premise setup is more accessible for students and hobbyists and avoids the complexity and cost of cloud-based robotics platforms.
- **Alternatives Considered**:
  - **Cloud-based lab (e.g., AWS RoboMaker)**: Powerful and scalable, but introduces significant cost and complexity. It is not ideal for a beginner-focused book.
  - **Hybrid model**: This adds complexity that would detract from the core learning objectives. The book focuses on the robotics and AI stack, not cloud infrastructure.
