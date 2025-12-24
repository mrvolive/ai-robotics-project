"""turtlebot_wall-following controller"""

from controller import Robot
from lidar import Lidar
from motors import Motors
from brain import Brain
from memory import Memory


class TurtlebotWallFollowing:
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
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.current_state = self.STATE_IDLE
        self.memory = Memory()

        self._init_devices()
        self.brain = Brain(self.memory, self.lidar)

    def _init_devices(self):
        """Initialize robot devices: Lidar and Motors."""
        self.lidar = Lidar(self.robot.getDevice("LDS-01"), self.timestep)
        self.motors = Motors(
            self.robot.getDevice("left wheel motor"),
            self.robot.getDevice("right wheel motor")
        )

    def _update_state(self, point_cloud):
        self.current_state = self.brain.think(
            point_cloud,
            self.current_state,
            self.COUNTER_MAX,
            self.OBSTACLE_THRESHOLD,
            self.WALL_FOLLOW_DIST,
            self.ALIGNMENT_THRESHOLD
        )

    def _get_target_velocities(self):
        """Map current state to desired linear (u) and angular (w) velocities."""
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
        """Main control loop."""
        while self.robot.step(self.timestep) != -1:
            # --- See ---
            point_cloud = self.lidar.get_range_image()

            # --- Think ---
            self._update_state(point_cloud)
            u_d, w_d = self._get_target_velocities()

            # --- Act ---
            self.motors.set_velocity(u_d, w_d)

            # Debug output
            print(
                f"State: {self.current_state:8s} | "
                f"Last turn: {self.memory.last_turn:8s} | "
                f"Action: {self.memory.current_action}"
            )


if __name__ == "__main__":
    controller = TurtlebotWallFollowing()
    controller.run()
