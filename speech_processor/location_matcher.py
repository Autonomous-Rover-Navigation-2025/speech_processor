import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sentence_transformers import SentenceTransformer, util

# Define your known UCI locations
UCI_LOCATIONS = [
    "library", "cafeteria", "admin building", "engineering lab", "student center", "gym", "classroom",
    "computer science department", "ARC", "dorm", "aldrich park", "lecture hall", "main entrance",
    "art gallery", "science library", "north park", "engineering tower", "next class", "robotics lab",
    "chemistry class", "front gate", "parkview building", "restroom", "multipurpose building",
    "administration", "university center", "south wing", "physics department", "research lab",
    "medical center", "verano place", "faculty office", "student union", "west entrance",
    "reception", "Palo Verde", "social science trailer", "innovation hub", "ICS", "project room",
    "engineering gateway", "alumni center", "registrar's office", "student health center", "middle earth",
    "mesa court", "infrastructure office", "brandywine", "bookstore", "bus stop", "science quad",
    "anteater plaza", "science hall", "info center", "bren events center", "humanities hall", "vdc",
    "cross cultural center", "tech repair", "financial aid office", "phoenix court", "DBH", "UTC", 
    "donald bren hall", "university town center", "anteater hill"
]

class LocationMatcherNode(Node):
    def __init__(self):
        super().__init__('location_matcher_node')

        # Initialize model and precompute embeddings
        self.model = SentenceTransformer("all-MiniLM-L6-v2")
        self.location_names = UCI_LOCATIONS
        self.location_embeddings = self.model.encode(self.location_names, convert_to_tensor=True)

        # ROS 2 I/O
        self.subscription = self.create_subscription(
            String,
            'escort_command',
            self.command_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'matched_location', 10)

        self.get_logger().info("LocationMatcherNode is ready.")

    def command_callback(self, msg: String):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Encode and match
        query_embedding = self.model.encode(command, convert_to_tensor=True)
        scores = util.cos_sim(query_embedding, self.location_embeddings)[0]
        best_match_index = scores.argmax().item()
        matched_location = self.location_names[best_match_index]

        self.get_logger().info(f"Matched location: {matched_location}")
        self.publisher.publish(String(data=matched_location))

def main(args=None):
    rclpy.init(args=args)
    node = LocationMatcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()