# Chapter 10: Isaac Sim Basic Scene Example

This directory contains a Python script (`basic_scene.py`) that demonstrates how to set up a basic simulation environment in NVIDIA Isaac Sim using its Python API.

## About the Script

The `basic_scene.py` script performs the following actions:

1.  **Initializes the Simulation**: It starts the Isaac Sim application.
2.  **Creates a World**: It sets up a new simulation world.
3.  **Adds a Ground Plane**: It adds a simple ground plane to the scene.
4.  **Loads the Robot**: It loads the `unitree_g1_sensors.urdf` model from `examples/ch09/` and places it in the scene.
5.  **Enters a Simulation Loop**: It continuously steps the simulation, which would allow physics and sensors to be simulated if the "PLAY" button is pressed in the UI.

## How to Run This Example

Unlike previous command-line examples, Isaac Sim scripts are typically run from within the Isaac Sim environment itself.

1.  **Launch Isaac Sim**: Start Isaac Sim from the NVIDIA Omniverse Launcher.

2.  **Open the Script Editor**:
    - Go to `Window` > `Script Editor`.
    - This will open a new window where you can write and execute Python code.

3.  **Update the URDF Path**:
    - **IMPORTANT**: Open the `basic_scene.py` file in a text editor.
    - Find the line `robot_urdf_path = "D:/spec_driven_development/physical-ai-and-humanoid-robotics/examples/ch09/unitree_g1_sensors.urdf"`.
    - You **MUST** change this to the absolute path of the `unitree_g1_sensors.urdf` file on your own computer.

4.  **Run the Script**:
    - In the Script Editor window in Isaac Sim, click `File` > `Open`.
    - Navigate to this directory (`examples/ch10/`) and select `basic_scene.py`.
    - The code will load into the editor.
    - Click the green "Run" button (a play icon) in the Script Editor toolbar.

After running the script, you will see the ground plane and the Unitree G1 robot appear in the simulation viewport. You can then press the main "PLAY" button at the top of the Isaac Sim interface to start the physics simulation.
