"""Action enumeration for robot behaviors during wall-following navigation.

This module defines the set of possible actions the robot can perform while
navigating through a maze without a map. Actions are used by the brain module
to coordinate complex behaviors that require multiple steps.
"""

from enum import Enum


class Action(Enum):
    """Enumeration of robot navigation actions.

    Attributes:
        NONE: No specific action, robot in normal operation mode
        TURN_90: Execute a 90-degree turn (left or right)
        ALIGNING_LEFT: Preparing to turn left, adjusting alignment
        ALIGNING_RIGHT: Preparing to turn right, adjusting alignment
        BACK_TO_INTERSECTION: Returning to the last intersection
    """
    NONE = ""
    TURN_90 = "90turn"
    ALIGNING_LEFT = "aligning_left"
    ALIGNING_RIGHT = "aligning_right"
    BACK_TO_INTERSECTION = "back_to_last_intersection"
