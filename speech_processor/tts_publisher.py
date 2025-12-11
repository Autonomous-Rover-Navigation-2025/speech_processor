#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import shlex

class TTSPublisher(Node):
    def __init__(self):
        super().__init__('tts_publisher')
        self.sub = self.create_subscription(String, '/speak_text', self.speak, 10)
        self.get_logger().info('TTS Publisher ready (Piper CLI mode)')

    def speak(self, msg):
        text = msg.data.strip()
        if not text:
            return

        try:
            safe_text = shlex.quote(text)

            cmd = [
                "bash", "-c",
                f"printf \"%s\" {safe_text} | /opt/piper/piper --model /models/piper/en_US-lessac-medium.onnx --output_file /tmp/tts.wav && aplay /tmp/tts.wav"
            ]
            # cmd = [
            # "bash", "-c",
            # (
            #     f'printf "%s" {safe_text} | '
            #     f'/opt/piper/piper --model /models/piper/en_US-lessac-medium.onnx '
            #     f'--output-raw | '
            #     f'aplay -r 22050 -f S16_LE -t raw -'
            # )
            # ]

            subprocess.run(cmd, check=True)
            self.get_logger().info(f"Spoken: {text}")

        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")

def main():
    rclpy.init()
    rclpy.spin(TTSPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
