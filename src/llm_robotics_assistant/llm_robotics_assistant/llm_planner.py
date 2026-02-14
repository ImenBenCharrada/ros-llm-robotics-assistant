import os
import json
from openai import OpenAI
from llm_robotics_assistant.schemas import TaskPlan

SYSTEM = """
You are a household robot task planner.

Allowed actions:
- navigate, detect_object, grasp, place, speak, wait

Allowed locations:
- kitchen, living_room, hallway, user, table

Rules:
- Always detect_object before grasp for the same object.
- Always grasp before place.
- place MUST include args.target (e.g. {"target":"table"}).
- Output ONLY by calling the tool submit_task_plan.
"""

def generate_plan_tool_call(task: str, model: str = "gpt-4.1-mini") -> dict:
    if not os.getenv("OPENAI_API_KEY"):
        raise RuntimeError("OPENAI_API_KEY is not set")

    client = OpenAI()

    tool = {
        "type": "function",
        "name": "submit_task_plan",
        "description": "Submit a TaskPlan that the robot can execute.",
        "parameters": TaskPlan.model_json_schema(),
    }


    resp = client.responses.create(
        model=model,
        input=[
            {"role": "system", "content": SYSTEM},
            {"role": "user", "content": task},
        ],
        tools=[tool],
        tool_choice="auto",
    )

    # The tool call is returned in resp.output as a function_call item. :contentReference[oaicite:2]{index=2}
    fc = None
    for item in resp.output:
        if getattr(item, "type", None) == "function_call" and getattr(item, "name", None) == "submit_task_plan":
            fc = item
            break

    if fc is None:
        # If model didn't call the tool, show the text for debugging
        raise RuntimeError(f"Model did not call submit_task_plan. Output text: {resp.output_text}")

    args = json.loads(fc.arguments)  # arguments are JSON-encoded in the tool call :contentReference[oaicite:3]{index=3}
    return args

def generate_plan(task: str) -> dict:
    # 1) get plan via tool calling
    plan_dict = generate_plan_tool_call(task)

    # 2) validate with pydantic (hard guarantee)
    TaskPlan.model_validate(plan_dict)

    return plan_dict
