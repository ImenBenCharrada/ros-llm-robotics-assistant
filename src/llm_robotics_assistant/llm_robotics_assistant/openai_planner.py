import os, json
from openai import OpenAI
from .schemas import TaskPlan
from .safety import validate_plan

SYSTEM = """You are a robotics planner.
Return a short plan using ONLY: navigate, detect_object, grasp, place, speak, wait.
Use allowed locations only: kitchen, living_room, hallway, user, table, charging_dock.
Always detect_object before grasp for the same object.
You MUST call submit_task_plan exactly once with the full plan.
"""

class OpenAIPlanner:
    def __init__(self, model="gpt-4.1-mini"):
        if not os.getenv("OPENAI_API_KEY"):
            raise RuntimeError("Set OPENAI_API_KEY")
        self.client = OpenAI()
        self.model = model

    def plan(self, task: str) -> TaskPlan:
        tool = {
            "type": "function",
            "function": {
                "name": "submit_task_plan",
                "description": "Submit the full TaskPlan JSON.",
                "parameters": TaskPlan.model_json_schema(),
                "strict": True,
            },
        }

        messages = [
            {"role": "system", "content": SYSTEM},
            {"role": "user", "content": task},
        ]

        for _ in range(3):
            resp = self.client.responses.create(
                model=self.model,
                input=messages,
                tools=[tool],
                tool_choice="auto",
            )

            call = next((x for x in resp.output if getattr(x, "type", "") == "function_call"), None)
            if not call:
                raise RuntimeError("Model did not call tool")

            args = json.loads(call.arguments)
            plan = TaskPlan.model_validate(args).validate_basic()
            violations = validate_plan(plan)

            if not violations:
                return plan

            messages.append({
                "role": "user",
                "content": "Fix these safety violations and call submit_task_plan again:\n"
                           + json.dumps([v.__dict__ for v in violations], indent=2)
            })

        raise RuntimeError("Failed to produce a safe plan")
