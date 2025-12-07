# Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate a LaunchDescription for bringing up the Nav2 stack.

    This launch file is a wrapper around the standard `bringup_launch.py`
    provided by the `nav2_bringup` package. It allows us to specify our own
    custom map and parameters file.
    """

    # Get the share directory for the nav2_bringup package
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # Get the share directory for this example package to find our custom files
    example_dir = os.path.dirname(os.path.realpath(__file__))

    # --- Declare Launch Arguments ---

    # Argument for the map file. Default is an empty string, assuming no map initially.
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(example_dir, 'maps', 'my_map.yaml'), # Placeholder path
        description='Full path to the map file to load.'
    )

    # Argument for the Nav2 parameters file.
    declare_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(example_dir, 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file.'
    )

    # Argument to enable simulation time. Essential for working with simulators.
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo or Isaac Sim) clock.'
    )

    # --- Include the Main Nav2 Bringup Launch File ---

    # This is the primary action. We are including another launch file and passing
    # our custom arguments to it.
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file')
        }.items(),
    )

    # --- Static TF Publisher for Map to Odom ---
    # In a real scenario, a localization component like AMCL would provide this transform.
    # For simple testing, a static transform publisher can be used.
    static_tf_pub_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )


    return LaunchDescription([
        declare_map_arg,
        declare_params_arg,
        declare_use_sim_time_arg,
        # static_tf_pub_node, # Uncomment for testing without a localization node
        nav2_bringup_cmd
    ])
