import json
import rclpy
from rclpy.node import Node
from llm_robotics_assistant_interfaces.srv import ExecuteAction

class NavNodeMock(Node):
    def __init__(self):
        super().__init__("nav_node_mock")
        self.srv = self.create_service(ExecuteAction, "/nav_node_mock/execute", self.on_execute)
        self.get_logger().info("Nav mock ready: /nav_node_mock/execute")

    def on_execute(self, request, response):
        action = json.loads(request.action_json)
        target = action["args"]["target"]
        response.success = True
        response.message = f"Navigated to {target}"
        response.result_json = json.dumps({"reached": target})
        return response

def main():
    rclpy.init()
    node = NavNodeMock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
