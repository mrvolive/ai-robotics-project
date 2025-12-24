"""Decision-making module for robot wall-following navigation.

This module implements the brain logic that processes sensor data and determines
the robot's next action. It uses a state machine approach to handle complex
navigation scenarios including obstacle avoidance, alignment, and maze solving.
"""

from actions import Action


class Brain:
    """Main decision-making controller for robot navigation.

    The Brain class processes sensor data from the lidar and encoders, stores
    relevant information in memory, and determines the next movement command
    based on the current situation. It implements a right-hand wall following
    algorithm for maze exploration.

    Attributes:
        memory: Memory instance storing robot state and navigation history
        lidar: Lidar instance providing distance measurements
        encoders: Encoders instance providing wheel position data
    """

    def __init__(self, memory, lidar, encoders):
        """Initialize the brain with required sensor and memory components.

        Args:
            memory: Memory instance for storing navigation state
            lidar: Lidar sensor instance for obstacle detection
            encoders: Wheel encoder instance for odometry
        """
        self.memory = memory
        self.lidar = lidar
        self.encoders = encoders

    def think(self, current_state, counter_max, obstacle_threshold, wall_follow_dist, alignment_threshold):
        """Main decision loop that determines the next robot action.

        This method implements the sense-think-act cycle. It first checks if
        an ongoing action needs to continue, then gathers sensor data, and
        finally decides on the appropriate movement command based on the
        current environment and robot state.

        Args:
            current_state: Current movement state of the robot
            counter_max: Maximum counter value for timed actions
            obstacle_threshold: Distance threshold for detecting obstacles (m)
            wall_follow_dist: Distance threshold for wall following (m)
            alignment_threshold: Threshold for checking wall alignment (m)

        Returns:
            str: Next movement command ("forward", "left", "right", "backward")
        """
        if self.memory.update_counter():
            return current_state

        self.memory.update_memory_count()
        # --- See ---
        self.memory.point_cloud = self.lidar.get_range_image()
        self.memory.left_encoder_value = self.encoders.get_left_encoder_value()
        self.memory.right_encoder_value = self.encoders.get_right_encoder_value()

        # --- Decide ---
        if self.memory.current_action == Action.ALIGNING_LEFT:
            self.memory.start_action(Action.TURN_90, counter_max / 2)
            return "left"

        if self.memory.current_action == Action.ALIGNING_RIGHT:
            self.memory.start_action(Action.TURN_90, counter_max / 2)
            return "right"

        if self.memory.current_action == Action.BACK_TO_INTERSECTION:
            return self._handle_back_to_intersection(obstacle_threshold)

        if self.memory.current_action == Action.TURN_90:
            self.memory.start_action(Action.NONE)
            return "forward"

        if self.lidar.is_obstacle_near(self.lidar.L_FRONT, obstacle_threshold):
            return self._handle_front_obstacle(counter_max, wall_follow_dist, alignment_threshold)

        return "forward"

    def _handle_back_to_intersection(self, obstacle_threshold):
        """Handle navigation when returning to the last intersection.

        When the robot needs to return to an intersection, this method checks
        for available paths and directs the robot accordingly. It uses the
        remembered turn direction to decide on the appropriate action.

        Args:
            obstacle_threshold: Distance threshold for detecting obstacles (m)

        Returns:
            str: Movement command ("left", "right", "backward")
        """
        if self.lidar.is_gap(self.lidar.L_LEFT, obstacle_threshold):
            if self.memory.last_turn == "left":
                self.memory.start_action(Action.TURN_90, 180)
                return "right"
            self.memory.start_action(Action.ALIGNING_LEFT, 27)
            return "left"
        elif self.lidar.is_gap(self.lidar.L_RIGHT, obstacle_threshold):
            self.memory.start_action(Action.ALIGNING_RIGHT, 27)
            return "right"
        return "backward"

    def _handle_front_obstacle(self, counter_max, wall_follow_dist, alignment_threshold):
        """Handle navigation when an obstacle is detected in front of the robot.

        This method implements the core wall-following logic. It first checks
        if the robot is properly aligned with the front wall, then determines
        which direction to turn based on the presence of walls on the left
        and right sides (right-hand wall following rule).

        Args:
            counter_max: Maximum counter value for timed actions
            wall_follow_dist: Distance threshold for detecting side walls (m)
            alignment_threshold: Threshold for checking front wall alignment (m)

        Returns:
            str: Movement command ("left", "right", "backward")
        """
        print("I've found a wall in front of me")
        offset = self.lidar.get_alignment_offset(self.lidar.L_FRONT)
        print(f'offset is : {offset}')
        if offset > alignment_threshold:
            print('not aligned correcting right')
            return "right"
        elif offset < -alignment_threshold:
            print('not aligned correcting left')
            return "left"
        elif self.lidar.is_obstacle_near(self.lidar.L_LEFT, wall_follow_dist):
            print('There is a wall on my left')
            if self.lidar.is_obstacle_near(self.lidar.L_RIGHT, wall_follow_dist):
                print('There is a wall on my right')
                self.memory.current_action = Action.BACK_TO_INTERSECTION
                return "backward"
            print('There is no wall on my right')
            self.memory.remember_turn("right")
            self.memory.start_action(Action.TURN_90, counter_max / 2)
            return "right"
        print('There is no wall on my left')
        self.memory.remember_turn("left")
        self.memory.start_action(Action.TURN_90, counter_max / 2)
        return "left"
