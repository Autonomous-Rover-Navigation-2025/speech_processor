#!/usr/bin/env python3
import os
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OllamaResponder(Node):
    def __init__(self):
        super().__init__('ollama_responder')

        # --- Configuration ---
        self.endpoint = self.declare_parameter(
            'endpoint',
            os.environ.get('OLLAMA_ENDPOINT', 'http://localhost:11434/api/generate')
        ).get_parameter_value().string_value

        self.model = self.declare_parameter(
            'model',
            os.environ.get('OLLAMA_MODEL', 'phi3')
        ).get_parameter_value().string_value

        self.temperature = self.declare_parameter('temperature', 0.3).get_parameter_value().double_value
        self.max_tokens = self.declare_parameter('max_tokens', 128).get_parameter_value().integer_value

        # --- ROS2 Publishers/Subscribers ---
        self.sub_text = self.create_subscription(String, '/llm_prompt', self.on_text, 10)
        self.pub_reply = self.create_publisher(String, '/speech_reply', 10)
        self.pub_tts = self.create_publisher(String, '/speak_text', 10)

        self.get_logger().info(
            f'🧠 OllamaResponder active at {self.endpoint} (model={self.model}, '
            f'temp={self.temperature}, max_tokens={self.max_tokens})'
        )

    def on_text(self, msg: String):
        prompt = msg.data.strip()
        if not prompt:
            return

        self.get_logger().info(f'🎙 Received LLM prompt: {prompt}')

        payload = {
            "model": self.model,
            "prompt": prompt,
            "stream": False,
            "options": {
                "temperature": self.temperature,
                "num_predict": self.max_tokens
            }
        }

        try:
            r = requests.post(self.endpoint, json=payload, timeout=120)
            r.raise_for_status()
            reply = r.json().get("response", "").strip()

            if not reply:
                self.get_logger().warn("⚠️ Empty response from Ollama")
                return

            # Publish to both text reply and TTS
            self.pub_reply.publish(String(data=reply))
            self.pub_tts.publish(String(data=reply))

            self.get_logger().info(f'💬 LLM reply: {reply[:120]}')

        except Exception as e:
            self.get_logger().error(f'❌ Ollama request failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OllamaResponder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
