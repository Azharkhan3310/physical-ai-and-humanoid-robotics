# Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """
    Generate a LaunchDescription for the Isaac ROS AprilTag pipeline.

    This launch file starts a container and loads the AprilTag composable node
    into it. This is a standard pattern for running Isaac ROS GEMs to ensure
    maximum performance through intra-process communication.
    """

    # Declare a launch argument for the physical size of the AprilTags
    # This allows the user to specify the tag size from the command line.
    declare_tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.05',  # Default size: 5cm
        description='The physical size of the AprilTag in meters.'
    )

    # Define the composable node for AprilTag detection
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        # Parameters for the AprilTag node
        parameters=[{
            'size': LaunchConfiguration('tag_size'),
            'max_tags': 32, # Maximum number of tags to detect in a single image
            'backend': 'CUDABACKEND' # Use the GPU-accelerated backend
        }],
        # Remap topics to match our robot's sensor outputs
        # Assumes the camera is publishing on /robot/image_raw and /robot/camera_info
        remappings=[
            ('image', '/robot/image_raw'),
            ('camera_info', '/robot/camera_info'),
            ('tag_detections', '/tag_detections') # Topic to publish detections on
        ]
    )

    # Define the container that will host the composable node(s)
    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[apriltag_node],
        output='screen'
    )

    return LaunchDescription([
        declare_tag_size_arg,
        apriltag_container
    ])
