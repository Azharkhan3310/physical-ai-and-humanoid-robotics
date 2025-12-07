# Chapter 13: Whisper Speech-to-Text ROS 2 Node

This directory contains a ROS 2 node (`whisper_node.py`) that performs speech-to-text transcription using OpenAI's Whisper model.

## About the Node

The `whisper_node.py` script creates a `WhisperNode` that:
1.  **Initializes ROS 2**: Sets up a standard ROS 2 node.
2.  **Loads the Whisper Model**: It loads a specified Whisper model size (defaulting to `base`) onto the appropriate device (GPU if available, otherwise CPU).
3.  **Subscribes to an Audio Topic**: It listens for incoming audio data on a specified topic (default: `/audio/audio_raw`). It assumes the data is in a format compatible with `audio_msgs/msg/AudioData`.
4.  **Transcribes Speech**: When it receives audio, it converts the data into a format Whisper can process and performs transcription.
5.  **Publishes the Result**: It publishes the resulting text string to a command topic (default: `/robot_command`).

## Prerequisites

1.  **Install `openai-whisper`**:
    ```bash
    pip install openai-whisper
    ```
2.  **Install PyTorch**:
    Follow the official instructions on the [PyTorch website](https://pytorch.org/get-started/locally/) to install PyTorch with CUDA support if you have an NVIDIA GPU.
3.  **Install an Audio Capture Node**:
    You need another ROS 2 node that captures audio from a microphone and publishes it to the `/audio/audio_raw` topic. A common package for this is [`audio_common`](http://wiki.ros.org/audio_common). You would typically run a node from this package to get audio data from a device like a ReSpeaker microphone array.
4.  **Install `ffmpeg`**: Whisper requires `ffmpeg` to be installed on your system.
    ```bash
    sudo apt update && sudo apt install ffmpeg
    ```

## How to Run This Node

1.  **Place in a ROS 2 Package**:
    To run this node, it must be part of a ROS 2 Python package. You would typically:
    -   Create a new package (e.g., `ros2 pkg create --build-type ament_python my_voice_control`).
    -   Place `whisper_node.py` inside the package's node directory.
    -   Add an entry point for the node in the package's `setup.py` file:
        ```python
        'console_scripts': [
            'whisper_node = my_voice_control.whisper_node:main',
        ],
        ```
    -   Build the package with `colcon build`.

2.  **Launch the System**:
    -   First, run your audio capture node to start publishing microphone data.
    -   Then, run the whisper node from your sourced workspace:
        ```bash
        # Run the node with default parameters
        ros2 run my_voice_control whisper_node
        
        # Or, run with custom parameters
        ros2 run my_voice_control whisper_node --ros-args -p model_size:='small'
        ```

3.  **Verify the Output**:
    -   Speak into the microphone.
    -   In another terminal, listen to the command topic to see the transcribed text:
        ```bash
        ros2 topic echo /robot_command
        ```
    -   You should see the words you speak appear as string messages.
