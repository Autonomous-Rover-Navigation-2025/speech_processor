import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from transformers import DistilBertTokenizerFast, DistilBertForSequenceClassification
import torch

class IntentClassifierNode(Node):
    def __init__(self):
        super().__init__('intent_classifier_node')

        self.model_path = '/workspaces/isaac_ros-dev/src/speech_processor/speech_processor/intent_model'


        # Load model relative to this file's path
        # package_dir = get_package_share_directory('speech_processor')
        # self.model_path = os.path.join(package_dir, 'speech_processor','intent_model')

        # Force local-only model loading
        self.tokenizer = DistilBertTokenizerFast.from_pretrained(self.model_path, local_files_only=True)
        self.model = DistilBertForSequenceClassification.from_pretrained(self.model_path, local_files_only=True)
        self.model.eval()

        self.subscription = self.create_subscription(
            String,
            '/asr_output',
            self.listener_callback,
            10
        )
        self.escort_pub = self.create_publisher(String, '/escort_command', 10)

        self.get_logger().info("Intent classifier node is running")

    def listener_callback(self, msg):
        text = msg.data.strip()
        inputs = self.tokenizer(text, return_tensors="pt", truncation=True, padding=True)

        with torch.no_grad():
            outputs = self.model(**inputs)
            pred_id = torch.argmax(outputs.logits, dim=-1).item()
            intent = self.model.config.id2label[pred_id]

        self.get_logger().info(f"Intent: '{intent}' for input: '{text}'")

        if intent.lower() == "escort":
            self.escort_pub.publish(String(data=text))
            self.get_logger().info(f"Sent to /escort_command: '{text}'")

def main(args=None):
    rclpy.init(args=args)
    node = IntentClassifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
