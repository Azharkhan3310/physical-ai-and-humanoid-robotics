---
id: lab-setup
sidebar_position: 1
title: Setting Up Your Lab
---

# Setting Up Your Lab

This chapter provides a detailed, step-by-step guide to setting up your development environment. This "Digital Twin Workstation" will be the foundation for all the projects in this book.

## Prerequisites

To follow this book effectively, you will need a workstation with the following minimum specifications:

-   **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish). While other Linux distributions or macOS may work, all instructions and examples are tested on Ubuntu 22.04.
-   **NVIDIA GPU**: An NVIDIA graphics card with at least 8GB of VRAM. An RTX 3070 or better is recommended. The examples in this book were developed and tested on an NVIDIA RTX 4070 Ti.
-   **RAM**: At least 32GB of RAM.
-   **Disk Space**: At least 100GB of free disk space.
-   **Internet Connection**: A stable internet connection for downloading software and dependencies.

## Step 1: Install NVIDIA Drivers

A correct and up-to-date NVIDIA driver installation is absolutely critical for running Isaac Sim and other GPU-accelerated machine learning tasks.

1.  **Add Proprietary GPU Drivers PPA**:
    This repository provides access to the latest NVIDIA drivers.
    ```bash
    sudo add-apt-repository ppa:graphics-drivers/ppa
    sudo apt update
    ```
2.  **Find Recommended Driver**:
    This command will scan your hardware and suggest the most compatible NVIDIA driver version.
    ```bash
    ubuntu-drivers devices
    ```
    You will see output similar to this (the version number may vary):
    ```
    driver   : nvidia-driver-535 - third-party free
    ```
3.  **Install the Driver**:
    Install the recommended driver package. Replace `<recommended-driver-version>` with the version you found in the previous step (e.g., `nvidia-driver-535`).
    ```bash
    sudo apt install <recommended-driver-version>
    # Example: sudo apt install nvidia-driver-535
    ```
4.  **Reboot Your System**: A reboot is required for the new kernel module (driver) to be loaded and active.
    ```bash
    sudo reboot
    ```
5.  **Verify Installation**: After your system reboots, open a terminal and run the following command. You should see detailed information about your NVIDIA GPU.
    ```bash
    nvidia-smi
    ```
    If this command shows information about your GPU, your drivers are correctly installed.

## Step 2: Install ROS 2 Humble

ROS 2 Humble Hawksbill is a Long-Term Support (LTS) release and will be the primary robotics middleware used throughout this book.

1.  **Set Locale**:
    Ensure your locale is set correctly to avoid issues with some ROS 2 tools.
    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```
2.  **Add ROS 2 APT Repository**:
    Add the official ROS 2 repository to your system's software sources.
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
3.  **Install ROS 2 Desktop**:
    This command installs the full ROS 2 desktop environment, including GUI tools.
    ```bash
    sudo apt update
    sudo apt install ros-humble-desktop
    ```
4.  **Source the Setup File**: Add the ROS 2 setup script to your `~/.bashrc` file so that it's automatically sourced in new terminals.
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    *Note: You may need to open a new terminal or run `source ~/.bashrc` again for changes to take effect.*
5.  **Install `colcon`**: `colcon` is the standard command-line tool for building ROS 2 packages.
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

## Step 3: Install NVIDIA Isaac Sim

NVIDIA Isaac Sim, built on Omniverse, is our primary simulation platform for AI and advanced robotics.

1.  **Download and Install Omniverse Launcher**:
    - Go to the [NVIDIA Omniverse website](https://www.nvidia.com/en-us/omniverse/).
    - Download the Omniverse Launcher for Linux.
    - Make the downloaded file executable and run it from your Downloads folder:
      ```bash
      chmod +x omniverse-launcher-linux.AppImage
      ./omniverse-launcher-linux.AppImage
      ```
    Follow the on-screen instructions to complete the installation of the launcher. You may need to create an NVIDIA account.
2.  **Install Isaac Sim from Launcher**:
    - Open the newly installed NVIDIA Omniverse Launcher.
    - Navigate to the "Exchange" tab.
    - Search for "Isaac Sim" and install the latest available version (ensure its driver requirements align with your installed NVIDIA driver).
    - Also in the "Exchange" tab, search for and install the "ROS 2 Humble Bridge" to enable communication between ROS 2 and Isaac Sim.

## Step 4: Install Gazebo (Ignition)

Gazebo (Ignition) will be used for lighter, more tightly integrated ROS 2 simulations.

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Step 5: Final Verification

It's crucial to verify that all components are correctly installed before proceeding.

1.  **Test ROS 2**: Open two separate terminals.
    - In Terminal 1: `ros2 run demo_nodes_cpp talker`
    - In Terminal 2: `ros2 run demo_nodes_py listener`
    - You should see the listener printing messages like `[INFO] [listener]: I heard: [Hello World: 1]` from the talker.
2.  **Test Gazebo**:
    - Open a terminal and run: `gazebo -v 4`
    - The Gazebo GUI should launch, displaying a default empty world.
3.  **Test Isaac Sim**:
    - Launch Isaac Sim from the NVIDIA Omniverse Launcher.
    - It should open without graphics errors, displaying the Isaac Sim interface.
    - Load a sample scene (e.g., "Isaac Examples" -> "Hello World").

Congratulations! Your development environment is now fully set up and ready for the book's projects.
