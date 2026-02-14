import json
import rclpy
from rclpy.node import Node
from llm_robotics_assistant_interfaces.msg import TaskPlan
from llm_robotics_assistant_interfaces.srv import ExecuteAction

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class DryRunExecutor(Node):
    def __init__(self):
        super().__init__("dry_run_executor")
        self.cb_group = ReentrantCallbackGroup()

        self.sub = self.create_subscription(TaskPlan, "/task_plan", self.on_plan, 10, callback_group=self.cb_group)

        self.nav = self.create_client(ExecuteAction, "/nav_node_mock/execute", callback_group=self.cb_group)
        self.vision = self.create_client(ExecuteAction, "/vision_node_mock/execute", callback_group=self.cb_group)
        self.manip = self.create_client(ExecuteAction, "/manip_node_mock/execute", callback_group=self.cb_group)

        self.get_logger().info("Waiting for mock services...")

        self.nav.wait_for_service()
        self.get_logger().info("Nav service ready")

        self.vision.wait_for_service()
        self.get_logger().info("Vision service ready")

        self.manip.wait_for_service()
        self.get_logger().info("Manip service ready")


        self.get_logger().info("Dry run executor ready. Waiting for /task_plan...")

        self.pending_plan = None
        self.timer = self.create_timer(0.1, self._tick, callback_group=self.cb_group)


    def call(self, client, action_dict):
        req = ExecuteAction.Request()
        req.action_json = json.dumps(action_dict)

        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"Service {client.srv_name} not available", {}

        fut = client.call_async(req)

        # prevent freezing forever
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)

        if fut.result() is None:
            return False, f"Service call timed out: {client.srv_name}", {}

        res = fut.result()
        result = json.loads(res.result_json) if res.result_json else {}
        return res.success, res.message, result

    def on_plan(self, msg):
        self.pending_plan = msg.plan_json

    def _tick(self):
        if self.pending_plan is None:
            return

        plan_json = self.pending_plan
        self.pending_plan = None

        plan = json.loads(plan_json)
        self.get_logger().info(f"Executing task: {plan['task']}")

        for step in plan["actions"]:
            t = step["type"]

            self.get_logger().info(f"About to run step {step.get('id')} type={t}")

            if t == "navigate":
                ok, m, _ = self.call(self.nav, step)
            elif t == "detect_object":
                ok, m, _ = self.call(self.vision, step)
            elif t in ["grasp", "place"]:
                ok, m, _ = self.call(self.manip, step)
            else:
                ok, m = True, f"Skipped {t}"

            self.get_logger().info(f"Step {step['id']} {t}: {'OK' if ok else 'FAIL'} - {m}")
            if not ok:
                self.get_logger().warn("Stopping plan due to failure.")
                break


def main():
    rclpy.init()
    node = DryRunExecutor()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
