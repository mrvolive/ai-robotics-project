class Lidar:
    # Lidar Angles (Indices for the point cloud array)
    L_LEFT = 90
    L_FRONT = 180
    L_RIGHT = 270
    L_BACK = 0

    def __init__(self, lidar, timestep):
        self.lidar = lidar
        self.lidar.enable(timestep)
        self.lidar.enablePointCloud()

    def get_range_image(self):
        return self.lidar.getRangeImage()

    def is_obstacle_near(self, point_cloud, angle: int, threshold: float) -> bool:
        values = []
        for i in range(-8, 8):
            idx = (angle + i) % 360
            val = point_cloud[idx]
            if val != float("inf"):
                values.append(val)

        if not values:
            return True

        # Using min distance as it's safer for wall following
        mean_dist = min(values)
        return mean_dist <= threshold

    def get_alignment_offset(self, point_cloud, angle: int) -> float:
        """
        Calculate offset between left and right rays of a given angle
        to determine perpendicular alignment.
        """
        left_laser = point_cloud[(angle - 1) % 360]
        right_laser = point_cloud[(angle + 1) % 360]
        return left_laser - right_laser

    def is_gap(self, point_cloud, angle: int, threshold: float) -> bool:
        """Check if there is a gap (no wall) at the given angle."""
        return not self.is_obstacle_near(point_cloud, angle, threshold)
