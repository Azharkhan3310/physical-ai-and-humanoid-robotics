# Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.
#
# This work is licensed under the NVIDIA Source Code License
# for Isaac Sim. To view a copy of this license, see the LICENSE file.

import carb
from omni.isaac.kit import SimulationApp

# This is the standard boilerplate for launching an Isaac Sim application
CONFIG = {
    "renderer": "RayTracedLighting",
    "headless": False,
}
simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import World
from omni.isaac.core.objects import GroundPlane
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.kit.commands


def main():
    """The main function that sets up the scene."""

    # Get a handle to the simulation world
    world = World()

    # Reset the simulation environment
    world.reset()

    # Add a ground plane to the scene
    GroundPlane(prim_path="/World/groundPlane", z_position=0)

    # --- Load the Robot URDF ---
    # Note: The path to the URDF should be absolute or relative to the Isaac Sim assets folder.
    # For a project, you would typically make this path configurable.
    # IMPORTANT: Update this path to the correct location of your URDF file on your system.
    robot_urdf_path = "D:/spec_driven_development/physical-ai-and-humanoid-robotics/examples/ch09/unitree_g1_sensors.urdf"

    # Add a reference to the robot's URDF file to the stage
    robot_prim_path = "/World/Robot"
    add_reference_to_stage(usd_path=robot_urdf_path, prim_path=robot_prim_path)
    
    # You could now apply physics properties, add controllers, etc.
    # For example, let's move the robot up slightly so it's not starting inside the ground plane.
    omni.kit.commands.execute(
        "TransformPrimSRT",
        path=robot_prim_path,
        new_translation=carb.Float3(0, 0, 0.5),
    )

    print("-------------------------------------------------")
    print("Basic Isaac Sim scene has been set up.")
    print("A ground plane and the robot URDF have been loaded.")
    print("Press PLAY to start the simulation.")
    print("-------------------------------------------------")

    # --- Simulation Loop ---
    # The world.step() function is what advances the simulation.
    # We can run it in a loop to keep the simulation running.
    while simulation_app.is_running():
        # This is where you would put your logic that needs to run every simulation step.
        world.step(render=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        carb.log_error(f"Error in main function: {e}")
    finally:
        # Shutdown the simulation application
        simulation_app.close()
