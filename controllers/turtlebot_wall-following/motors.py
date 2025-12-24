"""Motor control interface for differential drive robot.

This module provides control over the robot's wheel motors. It converts desired
linear and angular velocities into individual wheel speeds based on the robot's
physical parameters.
"""


class Motors:
    """Motor controller for differential drive robot.

    The Motors class manages the left and right wheel motors of a differential
    drive robot. It handles velocity control mode and performs the kinematic
    conversion from desired robot velocities to individual wheel speeds.

    Attributes:
        R: Radius of the wheels in meters (0.330 m)
        D: Distance between the wheels (track width) in meters (0.160 m)
        left_motor: Webots motor device for left wheel
        right_motor: Webots motor device for right wheel
    """

    # Robot Physical Parameters
    R = 0.330  # radius of the wheels [m]
    D = 0.160  # distance between the wheels [m]

    def __init__(self, left_motor, right_motor):
        """Initialize motors with the given motor devices.

        Sets both motors to velocity control mode by setting their position
        to infinity and initializes them with zero velocity.

        Args:
            left_motor: Webots motor device for the left wheel
            right_motor: Webots motor device for the right wheel
        """
        self.left_motor = left_motor
        self.right_motor = right_motor
        
        # Initialize motors for velocity control
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def set_velocity(self, u_d, w_d):
        """Set robot velocities by converting to wheel speeds.

        This method uses the differential drive kinematic equations to convert
        the desired linear velocity (u_d) and angular velocity (w_d) into the
        required left and right wheel angular velocities.

        Kinematic equations:
            wr = (2*u + D*w) / (2*R)  (right wheel)
            wl = (2*u - D*w) / (2*R)  (left wheel)

        Args:
            u_d: Desired linear velocity in meters/second (positive = forward)
            w_d: Desired angular velocity in radians/second (positive = counter-clockwise)
        """
        wr_d = (2 * u_d + self.D * w_d) / (2 * self.R)
        wl_d = (2 * u_d - self.D * w_d) / (2 * self.R)
        
        self.left_motor.setVelocity(float(wl_d))
        self.right_motor.setVelocity(float(wr_d))
