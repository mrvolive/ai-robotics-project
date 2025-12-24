from actions import Action


class Brain:
    def __init__(self, memory, lidar, encoders):
        self.memory = memory
        self.lidar = lidar
        self.encoders = encoders

    def think(self, current_state, counter_max, obstacle_threshold, wall_follow_dist, alignment_threshold):
        if self.memory.update_counter():
            return current_state

        self.memory.update_memory_count()
        # --- See ---
        self.memory.point_cloud = self.lidar.get_range_image()
        self.memory.left_encoder_value = self.encoders.get_left_encoder_value()
        self.memory.right_encoder_value = self.encoders.get_right_encoder_value()

        # --- Deciding ---
        if self.memory.current_action == Action.ALIGNING_LEFT:
            self.memory.start_action(Action.TURN_90, counter_max / 2)
            return "left"

        if self.memory.current_action == Action.ALIGNING_RIGHT:
            self.memory.start_action(Action.TURN_90, counter_max / 2)
            return "right"

        if self.memory.current_action == Action.BACK_TO_INTERSECTION:
            return self._handle_back_to_intersection(self.memory.point_cloud, obstacle_threshold)

        if self.memory.current_action == Action.TURN_90:
            self.memory.start_action(Action.NONE)
            return "forward"

        if self.lidar.is_obstacle_near(self.memory.point_cloud, self.lidar.L_FRONT, obstacle_threshold):
            return self._handle_front_obstacle(self.memory.point_cloud, counter_max, wall_follow_dist, alignment_threshold)

        return "forward"

    def _handle_back_to_intersection(self, point_cloud, obstacle_threshold):
        if self.lidar.is_gap(point_cloud, self.lidar.L_LEFT, obstacle_threshold):
            if self.memory.last_turn == "left":
                self.memory.start_action(Action.TURN_90, 180)
                return "right"
            self.memory.start_action(Action.ALIGNING_LEFT, 27)
            return "left"
        elif self.lidar.is_gap(point_cloud, self.lidar.L_RIGHT, obstacle_threshold):
            self.memory.start_action(Action.ALIGNING_RIGHT, 27)
            return "right"
        return "backward"

    def _handle_front_obstacle(self, point_cloud, counter_max, wall_follow_dist, alignment_threshold):
        print("I've found a wall in front of me")
        offset = self.lidar.get_alignment_offset(point_cloud, self.lidar.L_FRONT)
        print(f'offset is : {offset}')
        if offset > alignment_threshold:
            print('not aligned correcting right')
            return "right"
        elif offset < -alignment_threshold:
            print('not aligned correcting left')
            return "left"
        elif self.lidar.is_obstacle_near(point_cloud, self.lidar.L_LEFT, wall_follow_dist):
            print('There is a wall on my left')
            if self.lidar.is_obstacle_near(point_cloud, self.lidar.L_RIGHT, wall_follow_dist):
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
