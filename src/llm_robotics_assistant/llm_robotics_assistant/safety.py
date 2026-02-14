from typing import Dict, List, Tuple, Any
import re

ALLOWED_LOCATIONS = {"kitchen", "living_room", "hallway", "user", "table"}
MAX_STEPS = 12
_STOPWORDS = {"the", "a", "an", "to", "from", "in", "at", "into", "onto", "on", "of"}

def extract_requested_locations(task: str) -> List[str]:
    """
    Extract likely location tokens after prepositions like 'from'/'to'/'in'/'at'.
    Skips filler words ('the', 'a', 'an') and grabs the next meaningful token.
    Works for: 'from the garage', 'to the user', 'in the kitchen', etc.
    """
    tokens = re.findall(r"[a-zA-Z_]+", task.lower())

    preps = {"from", "to", "in", "at", "into", "onto"}
    locs: List[str] = []

    i = 0
    while i < len(tokens):
        if tokens[i] in preps:
            j = i + 1
            # skip filler words
            while j < len(tokens) and tokens[j] in {"the", "a", "an"}:
                j += 1
            if j < len(tokens):
                locs.append(tokens[j])
            i = j
        i += 1

    # de-duplicate while preserving order
    out = []
    seen = set()
    for x in locs:
        if x not in seen:
            seen.add(x)
            out.append(x)
    return out


def validate_task_plan_consistency(task: str, plan: Dict[str, Any]) -> List[str]:
    violations = []

    requested_locs = extract_requested_locations(task)

    if not requested_locs:
        return violations

    plan_targets = [
        a.get("args", {}).get("target")
        for a in plan.get("actions", [])
        if a.get("type") == "navigate"
    ]

    for loc in requested_locs:
        # Only enforce if it looks like a location token (not too short)
        if len(loc) < 3:
            continue
        if loc not in plan_targets:
            violations.append(f"Plan does not include required location: {loc}")

    return violations


def validate_plan(plan: Dict[str, Any]) -> List[str]:
    """Return list of human-readable safety violations. Empty list = safe."""
    violations: List[str] = []

    actions = plan.get("actions", [])
    if not isinstance(actions, list) or len(actions) == 0:
        return ["Plan has no actions"]

    if len(actions) > MAX_STEPS:
        violations.append(f"Plan too long: {len(actions)} steps (max {MAX_STEPS})")

    # Track detected objects and whether we are holding something
    detected = set()
    holding = None

    for i, a in enumerate(actions, start=1):
        t = a.get("type")
        args = a.get("args", {}) or {}

        # location constraints
        if t == "navigate":
            target = args.get("target")
            if target not in ALLOWED_LOCATIONS:
                violations.append(f"Step {i}: navigate target '{target}' is not allowed")

        if t == "place":
            target = args.get("target")
            if target not in ALLOWED_LOCATIONS:
                violations.append(f"Step {i}: place target '{target}' is not allowed")

        # ordering constraints
        if t == "detect_object":
            obj = str(args.get("object_name", "")).strip().lower()
            if not obj:
                violations.append(f"Step {i}: detect_object missing object_name")
            else:
                detected.add(obj)

        if t == "grasp":
            obj = str(args.get("object_name", "")).strip().lower()
            if not obj:
                violations.append(f"Step {i}: grasp missing object_name")
            else:
                if obj not in detected:
                    violations.append(f"Step {i}: grasp '{obj}' without detecting it first")
                if holding is not None:
                    violations.append(f"Step {i}: grasp while already holding '{holding}'")
                holding = obj

        if t == "place":
            if holding is None:
                violations.append(f"Step {i}: place while holding nothing")
            # If holding something, place ends the hold
            holding = None

    return violations
