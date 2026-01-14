"""Decision-making module for robot navigation using simple obstacle avoidance.

This module implements a simple obstacle avoidance logic based on checking
specific lidar angles, similar to the provided ROS example.
"""

import math


class Brain:
    """Main decision-making controller for robot navigation.
    
    Implements a simple logic: if the path ahead is clear, move forward;
    otherwise, stop and rotate.
    """

    LINEAR_SPEED = 2.0   # Increased from 0.5
    ANGULAR_SPEED = 2.0  # Increased from 0.5

    def __init__(self, memory, lidar, encoders):
        """Initialize the brain.

        Args:
            memory: Memory instance (unused in this logic)
            lidar: Lidar sensor instance
            encoders: Encoders instance (unused in this logic)
        """
        self.memory = memory
        self.lidar = lidar
        self.encoders = encoders

    def think(self):
        """Compute robot velocities based on lidar data.

        Returns:
            tuple: (linear_velocity, angular_velocity)
        """
        # Get raw lidar values
        ranges = self.lidar.get_range_image()
        
        # Safety check for array bounds
        if not ranges:
            return 0.0, 0.0

        # Define a front sector to check for obstacles
        # We check +/- 30 degrees around the front angle
        front_angle = self.lidar.L_FRONT
        sector_values = []
        for i in range(-30, 31):
            idx = (front_angle + i) % 360
            sector_values.append(ranges[idx])
        
        # Filter out invalid values (NaN)
        valid_distances = [x for x in sector_values if not math.isnan(x)]
        
        if not valid_distances:
            min_dist = 0.0 # Assume blocked if sensor fails
        else:
            min_dist = min(valid_distances)
        
        # Threshold
        safe_distance = 0.3
        
        # Logic
        if min_dist > safe_distance:
            return self.LINEAR_SPEED, 0.0
        else:
            return 0.0, self.ANGULAR_SPEED
