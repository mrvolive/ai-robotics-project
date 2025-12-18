"""turtlebot_wall-following controller"""

from controller import Robot


class TurtlebotWallFollowing:
    # --- Constants ---
    # Lidar Angles (Indices for the point cloud array)
    L_LEFT = 90
    L_FRONT = 180
    L_RIGHT = 270
    L_BACK = 0

    # Robot Physical Parameters
    R = 0.330  # radius of the wheels [m]
    D = 0.160  # distance between the wheels [m]

    # State Machine Thresholds
    COUNTER_MAX = 180
    OBSTACLE_THRESHOLD = 0.2
    WALL_FOLLOW_DIST = 0.3
    ALIGNMENT_THRESHOLD = 0.05

    # States
    STATE_IDLE = "idle"
    STATE_FORWARD = "forward"
    STATE_LEFT = "left"
    STATE_RIGHT = "right"
    STATE_BACKWARD = "backward"

    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # State Initialization
        self.current_state = self.STATE_IDLE
        self.current_action = ""
        self.counter = 0

        # Hardware Initialization
        self._init_devices()

    def _init_devices(self):
        """Initialize robot devices: Lidar and Motors."""
        # 2D-LiDAR
        self.lidar = self.robot.getDevice("LDS-01")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        # Motors
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def _convert_velocity_to_wheel_speed(self, u_d, w_d):
        """
        Converts desired linear (u_d) and angular (w_d) speeds to wheel speeds.
        Returns: (left_wheel_speed, right_wheel_speed)
        """
        wr_d = (2 * u_d + self.D * w_d) / (2 * self.R)
        wl_d = (2 * u_d - self.D * w_d) / (2 * self.R)
        return float(wl_d), float(wr_d)

    def _is_obstacle_near(self, point_cloud, angle: int, threshold: float) -> bool:
        """Check if an obstacle is within threshold at given angle."""
        return point_cloud[angle] <= threshold

    def _get_alignment_offset(self, point_cloud, angle: int) -> float:
        """
        Calculate offset between left and right rays of a given angle
        to determine perpendicular alignment.
        """
        left_laser = point_cloud[(angle - 1) % 360]
        right_laser = point_cloud[(angle + 1) % 360]
        return left_laser - right_laser

    def _update_state(self, point_cloud):
        """
        Determines the next state based on sensor data and current state.
        Implements the finite state machine logic.
        """
        # 1. Do nothing if there is a counter
        if self.counter > 0:
            self.counter -= 1
            return

        # 2. Handle '90turn' Action Sequence
        if self.current_action == "90turn":
            offset = self._get_alignment_offset(point_cloud, self.L_BACK)
            if offset > self.ALIGNMENT_THRESHOLD:
                self.current_state = self.STATE_LEFT
            elif offset < -self.ALIGNMENT_THRESHOLD:
                self.current_state = self.STATE_RIGHT
            else:
                self.current_action = ""
                self.current_state = self.STATE_FORWARD
            return

        # 3. Default direction logic when hitting a front wall
        if self._is_obstacle_near(point_cloud, self.L_FRONT, self.OBSTACLE_THRESHOLD):
            # Decide which way to turn based on left wall distance
            if point_cloud[self.L_LEFT] < self.WALL_FOLLOW_DIST:
                self.current_state = self.STATE_RIGHT
            else:
                self.current_state = self.STATE_LEFT

            # Initiate turn sequence
            self.counter = 0.5 * self.COUNTER_MAX
            self.current_action = "90turn"
        else:
            self.current_state = self.STATE_FORWARD

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
            point_cloud = self.lidar.getRangeImage()

            # --- Think ---
            self._update_state(point_cloud)
            u_d, w_d = self._get_target_velocities()

            # --- Act ---
            left_speed, right_speed = self._convert_velocity_to_wheel_speed(u_d, w_d)
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)

            # Debug output
            print(
                f"State: {self.current_state:8s} | "
                f"Front: {point_cloud[self.L_FRONT]:.2f} | "
                f"Left: {point_cloud[self.L_LEFT]:.2f} | "
                f"Action: {self.current_action}"
            )


if __name__ == "__main__":
    controller = TurtlebotWallFollowing()
    controller.run()
