import json
import rclpy
from rclpy.node import Node
from llm_robotics_assistant_interfaces.srv import ExecuteAction

class ManipNodeMock(Node):
    def __init__(self):
        super().__init__("manip_node_mock")
        self.held = None
        self.srv = self.create_service(ExecuteAction, "/manip_node_mock/execute", self.on_execute)
        self.get_logger().info("Manip mock ready: /manip_node_mock/execute")

    def on_execute(self, request, response):
        try:
            action = json.loads(request.action_json)
            t = action.get("type", "")

            if t == "grasp":
                obj = str(action.get("args", {}).get("object_name", "")).lower().strip()
                if self.held:
                    response.success = False
                    response.message = f"Already holding {self.held}"
                    response.result_json = "{}"
                    return response
                self.held = obj
                response.success = True
                response.message = f"Grasped {obj}"
                response.result_json = json.dumps({"held": obj})
                return response

            if t == "place":
                target = str(action.get("args", {}).get("target", "")).strip()
                if not target:
                    response.success = False
                    response.message = "place requires args.target"
                    response.result_json = "{}"
                    return response

                if not self.held:
                    response.success = False
                    response.message = "Nothing to place"
                    response.result_json = "{}"
                    return response

                obj = self.held
                self.held = None
                response.success = True
                response.message = f"Placed {obj} at {target}"
                response.result_json = json.dumps({"placed": obj, "target": target})
                return response

            response.success = False
            response.message = f"Unsupported action type: {t}"
            response.result_json = "{}"
            return response

        except Exception as e:
            self.get_logger().error(f"Manip exception: {e}")
            response.success = False
            response.message = f"Exception: {e}"
            response.result_json = "{}"
            return response


def main():
    rclpy.init()
    node = ManipNodeMock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
