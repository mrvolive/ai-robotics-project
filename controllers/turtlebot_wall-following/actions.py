from enum import Enum


class Action(Enum):
    NONE = ""
    TURN_90 = "90turn"
    ALIGNING_LEFT = "aligning_left"
    ALIGNING_RIGHT = "aligning_right"
    BACK_TO_INTERSECTION = "back_to_last_intersection"
