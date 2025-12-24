"""Wheel encoder interface for robot odometry.

This module provides access to the wheel encoders which track the rotational
position of each wheel. This information is used for odometry and can be used
to estimate the robot's position and orientation.
"""


class Encoders:
    """Interface to the robot's wheel encoders.

    The Encoders class manages the position sensors that measure wheel rotation.
    Each encoder returns the angular position of its wheel in radians, which
    can be used for distance estimation and movement tracking.

    Attributes:
        left_encoder: Webots position sensor for the left wheel
        right_encoder: Webots position sensor for the right wheel
    """

    def __init__(self, left_wheel_sensor, right_wheel_sensor, timestep):
        """Initialize the encoders with the given sensors and timestep.

        Args:
            left_wheel_sensor: Webots position sensor device for left wheel
            right_wheel_sensor: Webots position sensor device for right wheel
            timestep: Simulation timestep in milliseconds for sensor updates
        """
        self.left_encoder = left_wheel_sensor
        self.left_encoder.enable(timestep)
        self.right_encoder = right_wheel_sensor
        self.right_encoder.enable(timestep)

    def get_left_encoder_value(self):
        """Return left wheel encoder value in radians.

        The encoder value represents the cumulative angular position of the
        left wheel since the start of the simulation.

        Returns:
            float: Left wheel angular position in radians
        """
        return self.left_encoder.getValue()

    def get_right_encoder_value(self):
        """Return right wheel encoder value in radians.

        The encoder value represents the cumulative angular position of the
        right wheel since the start of the simulation.

        Returns:
            float: Right wheel angular position in radians
        """
        return self.right_encoder.getValue()
