# VLA Master Launch File
#
# This launch file brings up the entire Vision-Language-Action pipeline,
# integrating all the components we have built in previous chapters.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Generates the launch description for the full VLA pipeline.
    """
    
    # Assume all our custom nodes are in a package named 'my_voice_control'
    # and our Nav2 examples are in 'examples/ch12' relative to the project root.
    # In a real project, these paths would be more robustly handled,
    # for example by creating a dedicated 'my_robot_bringup' package.
    
    project_root = get_package_share_directory('my_voice_control') # Placeholder
    nav2_launch_file = os.path.join(project_root, '../../examples/ch12', 'nav2_bringup.launch.py')

    return LaunchDescription([
        
        # --- Declare Launch Arguments ---
        DeclareLaunchArgument(
            'model_size', default_value='small',
            description='Size of the Whisper model to use (e.g., tiny, base, small, medium, large).'
        ),
        
        # --- 1. Audio Capture ---
        # This node would capture audio from a microphone.
        # Replace 'audio_common' and 'audio_capture' with your actual audio package and node.
        Node(
            package='audio_common',
            executable='audio_capture',
            name='audio_capture_node',
            parameters=[{'device': ''}], # Specify your microphone device, e.g., 'hw:0,0'
            remappings=[('/audio/audio', '/audio/audio_raw')],
            output='screen'
        ),

        # --- 2. Whisper Transcription Node (from Chapter 13) ---
        Node(
            package='my_voice_control',
            executable='whisper_node',
            name='whisper_node',
            parameters=[{
                'model_size': LaunchConfiguration('model_size')
            }],
            output='screen'
        ),
        
        # --- 3. LLM Planner Node (from Chapter 14) ---
        # This node listens to the text from Whisper and creates a plan.
        Node(
            package='my_voice_control',
            executable='llm_planner_node',
            name='llm_planner_node',
            output='screen'
        ),

        # --- 4. Task Action Server ---
        # This is the crucial node that translates high-level tasks from the LLM
        # into low-level actions (like calling Nav2). This is a conceptual placeholder
        # as its implementation would be highly specific to the robot's capabilities.
        Node(
            package='my_voice_control',
            executable='task_action_server',
            name='task_action_server',
            output='screen'
        ),

        # --- 5. Navigation Stack (from Chapter 12) ---
        # Include the Nav2 bringup launch file.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={'use_sim_time': 'True'}.items()
        )
    ])
