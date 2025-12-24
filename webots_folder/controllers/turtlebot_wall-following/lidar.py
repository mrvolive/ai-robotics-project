"""Lidar sensor interface for obstacle detection and wall following.

This module provides access to the lidar sensor which measures distances
to obstacles in 360 degrees around the robot. It includes methods for
detecting obstacles, checking for gaps, and determining wall alignment.
"""


class Lidar:
    """Interface to the robot's lidar sensor.

    The Lidar class manages the lidar device which provides distance measurements
    in a 360-degree field of view. It offers methods to analyze the environment
    for obstacle detection, wall following, and alignment checking.

    Attributes:
        lidar: Webots lidar device
        L_LEFT: Index for left direction in point cloud (90 degrees)
        L_FRONT: Index for front direction in point cloud (180 degrees)
        L_RIGHT: Index for right direction in point cloud (270 degrees)
        L_BACK: Index for back direction in point cloud (0 degrees)
    """

    # Lidar Angles (Indices for the point cloud array)
    L_LEFT = 90
    L_FRONT = 180
    L_RIGHT = 270
    L_BACK = 0

    def __init__(self, lidar, timestep):
        """Initialize the lidar sensor with the given device and timestep.

        Args:
            lidar: Webots lidar device
            timestep: Simulation timestep in milliseconds for sensor updates
        """
        self.lidar = lidar
        self.lidar.enable(timestep)
        self.lidar.enablePointCloud()

    def get_range_image(self):
        """Get the current lidar range image.

        Returns:
            list: Array of 360 distance values, one for each degree.
                  Each value is the distance in meters, or float("inf")
                  if no obstacle is detected.
        """
        return self.lidar.getRangeImage()

    def is_obstacle_near(self, angle: int, threshold: float) -> bool:
        """Check if there is an obstacle near the specified angle.

        This method checks a small range around the specified angle for any
        obstacles within the threshold distance. It considers the minimum
        distance among all rays in the checked range.

        Args:
            angle: Main angle to check (0-359 degrees)
            threshold: Distance threshold in meters

        Returns:
            bool: True if an obstacle is detected within threshold distance,
                  False otherwise. Returns True if no valid measurements are
                  available (conservative approach).
        """
        point_cloud = self.get_range_image()
        values = []
        for i in range(-8, 8):
            idx = (angle + i) % 360
            val = point_cloud[idx]
            if val != float("inf"):
                values.append(val)

        if not values:
            return True

        # Using min distance as it's safer for wall following
        min_dist = min(values)
        return min_dist <= threshold

    def get_alignment_offset(self, angle: int) -> float:
        """Calculate offset between left and right rays for alignment checking.

        This method compares the distance measured by two adjacent rays
        (one degree left and one degree right of the specified angle) to
        determine if the robot is perpendicular to a wall.

        A positive value indicates the left side is farther from the wall
        than the right side, meaning the robot should turn left to align.
        A negative value indicates the opposite.

        Args:
            angle: Center angle to check (0-359 degrees)

        Returns:
            float: Offset distance (left - right) in meters.
                   Zero indicates good alignment.
        """
        point_cloud = self.get_range_image()
        left_laser = point_cloud[(angle - 1) % 360]
        right_laser = point_cloud[(angle + 1) % 360]
        return left_laser - right_laser

    def is_gap(self, angle: int, threshold: float) -> bool:
        """Check if there is a gap (no wall) at the given angle.

        This is the inverse of is_obstacle_near - it returns True when there
        is no obstacle within the threshold distance at the specified angle.

        Args:
            angle: Angle to check (0-359 degrees)
            threshold: Distance threshold in meters

        Returns:
            bool: True if there is a gap (no obstacle), False otherwise
        """
        return not self.is_obstacle_near(angle, threshold)
