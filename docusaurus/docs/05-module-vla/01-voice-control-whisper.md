---
id: voice-control-whisper
sidebar_position: 1
title: Voice Control with Whisper
---

# Chapter 13: Voice Control with Whisper

Welcome to the final and most exciting module: Vision-Language-Action (VLA). In this module, we will bridge the gap between human language and robot actions, enabling us to command our robot using natural voice commands.

The first step in this process is converting our speech into text that a machine can understand. For this, we will use OpenAI's Whisper, a state-of-the-art automatic speech recognition (ASR) model.

## What is Whisper?

Whisper is an ASR system trained by OpenAI on a massive dataset of diverse audio. It is capable of transcribing speech from a wide variety of languages and accents with high accuracy. It can be run locally on your own hardware, making it an excellent choice for robotics applications where privacy and low latency are concerns.

Key features:
-   **High Accuracy**: Approaches human-level robustness and accuracy in speech recognition.
-   **Multilingual**: Supports transcription in dozens of languages.
-   **Open Source**: The models and code are open source, allowing for local deployment.
-   **Multiple Model Sizes**: Whisper comes in various sizes, from `tiny` to `large`, allowing you to trade off between speed and accuracy depending on your hardware.

## The Voice Control Pipeline

Our goal is to create a ROS 2 node that can listen to a microphone, transcribe the audio using Whisper, and then publish the resulting text to a ROS 2 topic. This text will later be used by a Large Language Model (LLM) to generate a plan for the robot.

The pipeline looks like this:

1.  **Audio Input**: A ROS 2 node captures audio from a microphone (e.g., a ReSpeaker Mic Array). This is typically published as `audio_msgs/msg/AudioData`.
2.  **Whisper Transcription Node**: A new ROS 2 node subscribes to the audio topic.
3.  **Speech-to-Text**: When it receives audio, this node uses the Whisper library to transcribe it into a text string.
4.  **Text Publication**: The node publishes the transcribed text to a `/robot_command` topic as a `std_msgs/msg/String`.

## Building a Whisper ROS 2 Node

Building a ROS 2 node around Whisper is straightforward. Here's a high-level overview of the Python code structure:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_msgs.msg import AudioData

import whisper
import numpy as np

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_transcription_node')
        
        # Load the Whisper model
        # 'base' is a good starting point for a balance of speed and accuracy
        self.model = whisper.load_model("base")
        
        # Create a subscriber to the audio topic
        self.subscription = self.create_subscription(
            AudioData,
            '/audio/audio_raw', # Assumes an audio capture node is publishing here
            self.audio_callback,
            10)
            
        # Create a publisher for the transcribed text
        self.publisher = self.create_publisher(String, '/robot_command', 10)

    def audio_callback(self, msg):
        self.get_logger().info('Received audio data, attempting transcription...')
        
        # Convert the audio message to a NumPy array that Whisper can process
        # The exact conversion depends on the audio source format
        audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

        # Use Whisper to transcribe the audio
        result = self.model.transcribe(audio_data)
        
        transcribed_text = result['text']
        
        if transcribed_text:
            self.get_logger().info(f'Transcription: "{transcribed_text}"')
            
            # Publish the transcribed text
            text_msg = String()
            text_msg.data = transcribed_text
            self.publisher.publish(text_msg)

# main function and boilerplate to run the node
# ...
```

## Example: Whisper ROS 2 Node

A complete, well-documented ROS 2 node that implements the speech-to-text pipeline described above is available in the examples for this chapter. The README file includes full details on dependencies, setup, and how to run the node.

-   [**Chapter 13 Example: README**](/examples/ch13/README)
-   [**Chapter 13 Example: Python Node**](../../../examples/ch13/whisper_node.py)

## Next Steps

With this node, we have successfully converted human speech into a machine-readable format. The robot can now "hear" our commands. In the next chapter, we will take this text and feed it into a Large Language Model (LLM) to interpret the command and generate a high-level plan for the robot to execute.
