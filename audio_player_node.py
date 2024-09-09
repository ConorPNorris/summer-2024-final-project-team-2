import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class AudioPlayerNode(Node):
    def __init__(self):
        super().__init__('audio_player_node')
        self.subscription = self.create_subscription(
            String,
            'play_audio',
            self.listener_callback,
            10)
        self.get_logger().info('Audio Player Node is ready.')

    def listener_callback(self, msg):
        audio_file = msg.data
        self.get_logger().info(f'Playing audio: {audio_file}')
        try:
            # Use subprocess to call the sox play command
            subprocess.run(["play", audio_file], check=True)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to play sound: {e}")

def main(args=None):
    rclpy.init(args=args)
    audio_player_node = AudioPlayerNode()

    rclpy.spin(audio_player_node)

    audio_player_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
