import json
import rclpy
from rclpy.node import Node
from llm_robotics_assistant_interfaces.srv import ExecuteAction

# simple fake world
SCENE = {
    "kitchen": ["bottle", "mug"],
    "table": ["keys"]
}

class VisionNodeMock(Node):
    def __init__(self):
        super().__init__("vision_node_mock")
        self.current_location = "kitchen"
        self.srv = self.create_service(ExecuteAction, "/vision_node_mock/execute", self.on_execute)
        self.get_logger().info("Vision mock ready: /vision_node_mock/execute")

    def on_execute(self, request, response):
        action = json.loads(request.action_json)
        obj = action["args"]["object_name"].lower()

        if obj in SCENE.get(self.current_location, []):
            response.success = True
            response.message = f"Detected {obj}"
            response.result_json = json.dumps({"detections": [{"label": obj, "confidence": 0.9}]})
        else:
            response.success = False
            response.message = f"{obj} not found"
            response.result_json = json.dumps({"detections": []})
        return response

def main():
    rclpy.init()
    node = VisionNodeMock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
