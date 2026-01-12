"""Memory module for robot state and navigation history.

This module provides the memory system that stores the robot's current state,
navigation history, and temporary data needed for multi-step actions. It allows
the robot to remember its last turn direction and track the progress of ongoing
actions.
"""

from actions import Action


class Memory:
    """Memory system for storing robot state and navigation history.

    The Memory class maintains all the persistent state information needed by
    the robot during navigation. This includes the current action, last turn
    direction, sensor data snapshots, and counters for timed actions.

    Attributes:
        current_action: Current Action being performed
        last_turn: Direction of last turn ("left", "right", or "")
        point_cloud: Latest lidar range image snapshot
        left_encoder_value: Latest left wheel encoder reading
        right_encoder_value: Latest right wheel encoder reading
        counter: Countdown timer for ongoing actions
        memory_count: Countdown for remembering last turn direction
    """

    def __init__(self):
        """Initialize memory with default values.

        Sets all state variables to their initial values with no action,
        no remembered turn, and zero counters.
        """
        self.current_action = Action.NONE
        self.last_turn = ""
        self.point_cloud = []
        self.left_encoder_value = 0
        self.right_encoder_value = 0
        self.counter = 0
        self.memory_count = 0
        self.turn_start_left = 0.0
        self.turn_start_right = 0.0

    def remember_turn(self, turn):
        """Remember the last turn direction for future decision making.

        This method stores the direction of the last turn taken, which is used
        by the navigation algorithm to make decisions at intersections. The
        remembered value persists for a limited time (controlled by memory_count).

        Args:
            turn: Turn direction to remember ("left" or "right")
        """
        self.last_turn = turn
        self.memory_count = 100

    def start_action(self, action, duration=0):
        """Start a new action with optional duration timer.

        This method sets the current action and optionally starts a countdown
        timer. Actions with a duration timer will continue for that many
        simulation steps before the next decision is made.

        Args:
            action: Action to start (from Action enum)
            duration: Number of simulation steps to maintain this action (default: 0)
        """
        self.current_action = action
        if duration > 0:
            self.counter = duration

    def update_counter(self):
        """Update the action counter and check if action is still in progress.

        Decrements the counter by one if it is greater than zero. This is called
        each simulation step to track the progress of timed actions.

        Returns:
            bool: True if counter > 0 (action still in progress), False otherwise
        """
        if self.counter > 0:
            self.counter -= 1
            return True
        return False

    def update_memory_count(self):
        """Update the memory counter and clear last turn when expired.

        Decrements the memory_count by one if it is greater than zero. When the
        count reaches zero, clears the remembered last_turn value. This prevents
        old turn information from influencing navigation decisions indefinitely.
        """
        if self.memory_count > 0:
            self.memory_count -= 1
        if self.memory_count <= 0:
            self.last_turn = ""
