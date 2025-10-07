import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('speaker_test_script')
        self.publisher_ = self.create_publisher(String, 'speak_text', 10)

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {text}')

def main(args=None):
    rclpy.init(args=args)
    node = Talker()

    try:
        while rclpy.ok():
            user_input = input("Enter text to publish (Ctrl+C to quit): ")
            node.publish_text(user_input)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down the speaker_test_script ...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()