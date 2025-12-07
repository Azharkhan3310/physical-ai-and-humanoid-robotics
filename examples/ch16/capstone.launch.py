# Capstone Project Master Launch File
#
# This launch file brings up the entire Vision-Language-Action pipeline
# for the final capstone project.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Generates the launch description for the full capstone project.
    """
    
    # In a real project, you would create a ROS 2 package for the capstone
    # and use get_package_share_directory to find files. For this book,
    # we assume the launch file is run from the project root and paths are relative.
    
    # Path to the Nav2 launch file from a previous chapter
    nav2_launch_file = os.path.join(
        os.getcwd(), 'examples/ch12/nav2_bringup.launch.py'
    )
    
    # Path to the custom nodes in this package
    # (Assuming this launch file is in a package that includes the nodes)
    pkg_dir = get_package_share_directory('my_voice_control') # Placeholder

    return LaunchDescription([
        
        # --- Declare Launch Arguments ---
        DeclareLaunchArgument(
            'model_size', default_value='base',
            description='Size of the Whisper model to use (e.g., tiny, base, small, medium, large).'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation (Gazebo or Isaac Sim) clock.'
        ),
        
        # --- 1. Simulation Environment ---
        # In a real scenario, you would also launch Isaac Sim or Gazebo here.
        # For this example, we assume the simulation is started manually.

        # --- 2. VLA Pipeline Nodes ---

        # Audio Capture Node
        Node(
            package='audio_common',
            executable='audio_capture',
            name='audio_capture_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # Whisper Transcription Node
        Node(
            package='my_voice_control',
            executable='whisper_node', # Entry point from setup.py
            name='whisper_node',
            parameters=[
                {'model_size': LaunchConfiguration('model_size')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),
        
        # LLM Planner Node
        Node(
            package='my_voice_control',
            executable='llm_planner_node', # Entry point from setup.py
            name='llm_planner_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # Task Action Server - The key integration piece
        Node(
            package='my_voice_control',
            executable='task_action_server', # Entry point from setup.py
            name='task_action_server',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # --- 3. Navigation Stack ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        )
    ])
