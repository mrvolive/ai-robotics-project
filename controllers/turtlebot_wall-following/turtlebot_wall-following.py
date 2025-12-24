"""Turtlebot wall-following controller for mapless navigation.

This module implements the main controller for a Turtlebot robot that navigates
through a maze using wall-following without a prior map. The controller uses a
right-hand wall following algorithm with lidar-based obstacle detection and
encoders for odometry.
"""

from brain import Brain
from controller import Robot
from encoders import Encoders
from lidar import Lidar
from memory import Memory
from motors import Motors


class TurtlebotWallFollowing:
    """Main controller class for Turtlebot wall-following navigation.

    This class orchestrates all components of the robot control system including
    the brain (decision making), lidar (obstacle detection), motors (actuation),
    encoders (odometry), and memory (state management). It implements the main
    control loop that runs until the simulation ends.

    Attributes:
        COUNTER_MAX: Maximum counter value for timed actions (180 steps)
        OBSTACLE_THRESHOLD: Distance threshold for obstacle detection (0.2 m)
        WALL_FOLLOW_DIST: Distance threshold for detecting walls to follow (0.30 m)
        ALIGNMENT_THRESHOLD: Distance threshold for wall alignment check (0.01 m)
        STATE_IDLE: State identifier for idle/stopped
        STATE_FORWARD: State identifier for forward motion
        STATE_LEFT: State identifier for turning left
        STATE_RIGHT: State identifier for turning right
        STATE_BACKWARD: State identifier for backward motion
        robot: Webots Robot instance
        timestep: Simulation timestep in milliseconds
        current_state: Current movement state of the robot
        memory: Memory instance for storing state and history
        lidar: Lidar sensor instance
        motors: Motor control instance
        encoders: Wheel encoder instance
        brain: Decision-making brain instance
    """

    # --- Constants ---
    # State Machine Thresholds
    COUNTER_MAX = 180
    OBSTACLE_THRESHOLD = 0.2
    WALL_FOLLOW_DIST = 0.30
    ALIGNMENT_THRESHOLD = 0.01

    # States
    STATE_IDLE = "idle"
    STATE_FORWARD = "forward"
    STATE_LEFT = "left"
    STATE_RIGHT = "right"
    STATE_BACKWARD = "backward"

    def __init__(self):
        """Initialize the robot controller and all subsystems.

        Creates the Webots Robot instance, initializes the simulation timestep,
        creates the memory system, and initializes all robot devices.
        Also creates the brain for decision making.
        """
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.current_state = self.STATE_IDLE
        self.memory = Memory()

        self._init_devices()
        self.brain = Brain(self.memory, self.lidar, self.encoders)

    def _init_devices(self):
        """Initialize robot devices: Lidar, Motors, and Encoders.

        Retrieves the required devices from the Webots simulation and creates
        the corresponding wrapper instances for each device.
        """
        self.lidar = Lidar(self.robot.getDevice("LDS-01"), self.timestep)
        self.motors = Motors(
            self.robot.getDevice("left wheel motor"),
            self.robot.getDevice("right wheel motor"),
        )
        self.encoders = Encoders(
            self.robot.getDevice("left wheel sensor"),
            self.robot.getDevice("right wheel sensor"),
            self.timestep,
        )

    def _update_state(self):
        """Update the current movement state based on brain decision.

        Calls the brain's think method with current state and thresholds to
        determine the next movement command. Updates the current_state accordingly.
        """
        self.current_state = self.brain.think(
            self.current_state,
            self.COUNTER_MAX,
            self.OBSTACLE_THRESHOLD,
            self.WALL_FOLLOW_DIST,
            self.ALIGNMENT_THRESHOLD,
        )

    def _get_target_velocities(self):
        """Map current state to desired linear (u) and angular (w) velocities.

        Converts the current movement state into the corresponding linear and
        angular velocity commands for the differential drive robot.

        Returns:
            tuple: (linear_velocity, angular_velocity) where:
                - linear_velocity is in m/s (positive = forward)
                - angular_velocity is in rad/s (positive = counter-clockwise)
        """
        if self.current_state == self.STATE_FORWARD:
            return 2.0, 0.0
        elif self.current_state == self.STATE_LEFT:
            return 0.0, 6.0
        elif self.current_state == self.STATE_RIGHT:
            return 0.0, -6.0
        elif self.current_state == self.STATE_BACKWARD:
            return -2.0, 0.0

        # Idle or unknown
        return 0.0, 0.0

    def run(self):
        """Main control loop.

        Executes the sense-think-act cycle for each simulation step:
        1. Think: Update state based on brain decision
        2. Get target velocities based on state
        3. Act: Set motor velocities
        4. Print debug information

        The loop continues until the simulation ends.
        """
        while self.robot.step(self.timestep) != -1:
            # --- Think ---
            self._update_state()
            u_d, w_d = self._get_target_velocities()

            # --- Act ---
            self.motors.set_velocity(u_d, w_d)

            # Debug output
            print(
                f"State: {self.current_state:8s} | "
                # f"Last turn: {self.memory.last_turn:8s} | "
                f"encoders_values: {self.memory.left_encoder_value:2f},{self.memory.right_encoder_value:2f} | "
                f"Action: {self.memory.current_action}"
            )


if __name__ == "__main__":
    controller = TurtlebotWallFollowing()
    controller.run()
