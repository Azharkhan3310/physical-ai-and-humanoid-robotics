import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_msgs.msg import AudioData  # Assuming a custom or standard audio message type

import whisper
import numpy as np
import torch

class WhisperNode(Node):
    """
    A ROS 2 node that listens to an audio topic, transcribes the speech using
    OpenAI's Whisper, and publishes the resulting text.
    """
    def __init__(self):
        super().__init__('whisper_transcription_node')

        # --- Parameters ---
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('audio_topic', '/audio/audio_raw')
        self.declare_parameter('command_topic', '/robot_command')

        model_size = self.get_parameter('model_size').get_parameter_value().string_value
        audio_topic = self.get_parameter('audio_topic').get_parameter_value().string_value
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value

        # Check for CUDA availability
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        # --- Whisper Model ---
        self.get_logger().info(f"Loading Whisper model '{model_size}'...")
        try:
            self.model = whisper.load_model(model_size, device=self.device)
            self.get_logger().info("Whisper model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load Whisper model: {e}")
            # Exit the node if the model fails to load
            raise e

        # --- ROS 2 Communication ---
        self.subscription = self.create_subscription(
            AudioData,
            audio_topic,
            self.audio_callback,
            10  # QoS profile depth
        )
        self.publisher = self.create_publisher(
            String,
            command_topic,
            10  # QoS profile depth
        )

        self.get_logger().info(f"Listening for audio on '{audio_topic}'...")
        self.get_logger().info(f"Publishing commands to '{command_topic}'...")

    def audio_callback(self, msg: AudioData):
        """
        Callback function for the audio subscriber.
        This function is called whenever a new audio message is received.
        """
        self.get_logger().info('Received audio data, attempting transcription...')

        # Convert the audio message data (bytes) to a NumPy array
        # Whisper expects a 16-bit signed integer NumPy array, normalized to [-1, 1]
        try:
            audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
        except Exception as e:
            self.get_logger().error(f"Failed to process audio data: {e}")
            return

        # Use Whisper to transcribe the audio
        try:
            result = self.model.transcribe(audio_data, fp16=self.device=="cuda")
            transcribed_text = result['text'].strip()
        except Exception as e:
            self.get_logger().error(f"Whisper transcription failed: {e}")
            return

        # Publish the transcribed text if it's not empty
        if transcribed_text:
            self.get_logger().info(f'Transcription: "{transcribed_text}"')
            
            text_msg = String()
            text_msg.data = transcribed_text
            self.publisher.publish(text_msg)
        else:
            self.get_logger().info("Transcription result is empty.")


def main(args=None):
    rclpy.init(args=args)
    try:
        whisper_node = WhisperNode()
        rclpy.spin(whisper_node)
    except Exception as e:
        rclpy.logging.get_logger("main").error(f"An error occurred: {e}")
    finally:
        # Destroy the node explicitly
        if 'whisper_node' in locals() and rclpy.ok():
            whisper_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
