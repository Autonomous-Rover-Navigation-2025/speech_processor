import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3

class TextToSpeechNode(Node):
    def __init__(self):
        super().__init__('text_to_speech_node')

        # Initialize the pyttsx3 engine
        self.engine = pyttsx3.init()

        # Set American English female voice if available
        voices = self.engine.getProperty('voices')
        for voice in voices:
            if "English (America)" in voice.name or "en_US" in voice.id:
                self.engine.setProperty('voice', voice.id)
                break
        else:
            self.get_logger().info("American English voice not found, using default.")

        # Set the speech rate and volume
        self.engine.setProperty('rate', 150)
        self.engine.setProperty('volume', 0.9)

        # Subscribe to the `/speak_text` topic to receive text messages
        self.subscription = self.create_subscription(
            String,
            'speak_text',
            self.speak_callback,
            10
        )
        self.get_logger().info("Text-to-Speech Node has started, waiting for text input...")

    def speak_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Speaking: '{text}'")

        # Speak the received text
        self.engine.say(text)
        self.engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)
    tts_node = TextToSpeechNode()
    
    try:
        rclpy.spin(tts_node)
    except KeyboardInterrupt:
        pass
    finally:
        tts_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

