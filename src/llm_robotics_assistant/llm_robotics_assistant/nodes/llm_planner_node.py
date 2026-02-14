import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from llm_robotics_assistant.llm_planner import generate_plan
from llm_robotics_assistant_interfaces.msg import TaskPlan as TaskPlanMsg
from llm_robotics_assistant.safety import (
    validate_plan,
    validate_task_plan_consistency
)


class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__("llm_planner_node")

        # Subscribe to natural language commands
        self.sub = self.create_subscription(
            String,
            "/nl_command",
            self.on_command,
            10
        )

        # Publish task plans
        self.pub = self.create_publisher(
            TaskPlanMsg,
            "/task_plan",
            10
        )

        self.get_logger().info("LLM Planner Node Ready!")

    def on_command(self, msg: String):
        task = msg.data
        self.get_logger().info(f"Received task: {task}")

        try:
            plan = generate_plan(task)

            intent_violations = validate_task_plan_consistency(task, plan)

            if intent_violations:
                self.get_logger().warn("Intent mismatch:\n- " + "\n- ".join(intent_violations))

                safe_plan = {
                    "task": task,
                    "actions": [
                        {
                            "id": "1",
                            "type": "speak",
                            "args": {"text": "I cannot safely complete that request."}
                        }
                    ]
                }

                out = TaskPlanMsg()
                out.plan_json = json.dumps(safe_plan)
                self.pub.publish(out)
                self.get_logger().info("Published SAFE fallback plan.")
                return


        except Exception as e:
            self.get_logger().error(f"LLM failed: {e}")
            return

        # Hard check: place must have args.target
        for a in plan.get("actions", []):
            if a.get("type") == "place":
                a.setdefault("args", {})
                if "target" not in a["args"]:
                    self.get_logger().error("Invalid plan: place missing args.target")
                    return

        out = TaskPlanMsg()
        out.plan_json = json.dumps(plan)
        self.pub.publish(out)
        self.get_logger().info("Published task plan!")


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
