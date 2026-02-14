from typing import List, Literal, Union
from pydantic import BaseModel, Field, ConfigDict


# ---- Action argument models (strict) ----
class NavigateArgs(BaseModel):
    model_config = ConfigDict(extra="forbid")
    target: Literal["kitchen", "living_room", "hallway", "user", "table"]

class DetectArgs(BaseModel):
    model_config = ConfigDict(extra="forbid")
    object_name: str = Field(min_length=1)

class GraspArgs(BaseModel):
    model_config = ConfigDict(extra="forbid")
    object_name: str = Field(min_length=1)

class PlaceArgs(BaseModel):
    model_config = ConfigDict(extra="forbid")
    target: Literal["table", "user", "kitchen", "living_room", "hallway"]

class SpeakArgs(BaseModel):
    model_config = ConfigDict(extra="forbid")
    text: str = Field(min_length=1)

class WaitArgs(BaseModel):
    model_config = ConfigDict(extra="forbid")
    seconds: float = Field(gt=0.0, le=30.0)


# ---- Discriminated action union (type decides args schema) ----
class NavigateAction(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: str
    type: Literal["navigate"]
    args: NavigateArgs

class DetectAction(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: str
    type: Literal["detect_object"]
    args: DetectArgs

class GraspAction(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: str
    type: Literal["grasp"]
    args: GraspArgs

class PlaceAction(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: str
    type: Literal["place"]
    args: PlaceArgs

class SpeakAction(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: str
    type: Literal["speak"]
    args: SpeakArgs

class WaitAction(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: str
    type: Literal["wait"]
    args: WaitArgs


Action = Union[
    NavigateAction,
    DetectAction,
    GraspAction,
    PlaceAction,
    SpeakAction,
    WaitAction,
]


class TaskPlan(BaseModel):
    model_config = ConfigDict(extra="forbid")
    task: str
    actions: List[Action] = Field(..., min_length=1)
