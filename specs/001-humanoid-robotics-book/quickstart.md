# Quickstart: Development Environment Setup

This guide provides the steps to set up a complete development environment for the "Physical AI & Humanoid Robotics Book" on an Ubuntu 22.04 workstation.

## Prerequisites

- A workstation with Ubuntu 22.04 installed.
- NVIDIA GPU with at least 8GB of VRAM (RTX 3070 or better recommended, RTX 4070 Ti as specified in the book).
- At least 32GB of RAM.
- At least 100GB of free disk space.
- An internet connection.

## Step 1: Install NVIDIA Drivers

A correct NVIDIA driver installation is critical for both Isaac Sim and machine learning tasks.

1.  **Add Proprietary GPU Drivers PPA**:
    ```bash
    sudo add-apt-repository ppa:graphics-drivers/ppa
    sudo apt update
    ```
2.  **Find Recommended Driver**:
    ```bash
    ubuntu-drivers devices
    ```
    This will list the recommended driver for your GPU (e.g., `nvidia-driver-535`).
3.  **Install the Driver**:
    ```bash
    sudo apt install <recommended-driver-version>
    # Example: sudo apt install nvidia-driver-535
    ```
4.  **Reboot**: A reboot is required to load the new driver.
    ```bash
    sudo reboot
    ```
5.  **Verify Installation**: After rebooting, run the following command. It should show your GPU details.
    ```bash
    nvidia-smi
    ```

## Step 2: Install ROS 2 Humble

We will install ROS 2 Humble Hawksbill, the current LTS release.

1.  **Set Locale**:
    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```
2.  **Add ROS 2 APT Repository**:
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
3.  **Install ROS 2 Desktop**:
    ```bash
    sudo apt update
    sudo apt install ros-humble-desktop
    ```
4.  **Source the Setup File**: Add the ROS 2 setup script to your shell startup script.
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
5.  **Install `colcon`**: This is the standard build tool for ROS 2 packages.
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

## Step 3: Install NVIDIA Isaac Sim

Isaac Sim requires the Omniverse Launcher.

1.  **Download and Install Omniverse Launcher**:
    - Go to the [NVIDIA Omniverse website](https://www.nvidia.com/en-us/omniverse/).
    - Download the Omniverse Launcher for Linux.
    - Make the downloaded file executable and run it:
      ```bash
      chmod +x omniverse-launcher-linux.AppImage
      ./omniverse-launcher-linux.AppImage
      ```
2.  **Install Isaac Sim from Launcher**:
    - Open the Omniverse Launcher.
    - Go to the "Exchange" tab.
    - Search for "Isaac Sim" and install the latest version (ensure it is compatible with your driver).
    - Also in the Exchange, search for and install "ROS 2 Humble Bridge" to enable communication.

## Step 4: Install Gazebo (Ignition)

ROS 2 Humble uses the Gazebo simulator, which was rebranded from Ignition.

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Step 5: Final Verification

1.  **Test ROS 2**: Open a new terminal and run the talker/listener example.
    - Terminal 1: `ros2 run demo_nodes_cpp talker`
    - Terminal 2: `ros2 run demo_nodes_py listener`
    - You should see the listener printing messages from the talker.
2.  **Test Gazebo**:
    - `gazebo -v 4`
    - The Gazebo GUI should launch.
3.  **Test Isaac Sim**:
    - Launch Isaac Sim from the Omniverse Launcher.
    - It should open without graphics errors.

Your development environment is now set up and ready for the book's projects.
