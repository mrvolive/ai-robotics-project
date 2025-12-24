class Memory:
    def __init__(self):
        self.current_action = ""
        self.last_turn = ""
        self.counter = 0
        self.memory_count = 0

    def remember_turn(self, turn):
        self.last_turn = turn
        self.memory_count = 100

    def start_action(self, action, duration=0):
        self.current_action = action
        if duration > 0:
            self.counter = duration

    def update_counter(self):
        if self.counter > 0:
            self.counter -= 1
            return True
        return False

    def update_memory_count(self):
        if self.memory_count > 0:
            self.memory_count -= 1
        if self.memory_count <= 0:
            self.last_turn = ""
