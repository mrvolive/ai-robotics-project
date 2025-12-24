class Encoders:
    def __init__(self, left_wheel_sensor, right_wheel_sensor, timestep):
        self.left_encoder = left_wheel_sensor
        self.left_encoder.enable(timestep)
        self.right_encoder = right_wheel_sensor
        self.right_encoder.enable(timestep)

    def get_left_encoder_value(self):
        """ return left wheel encoder value in radians """
        return self.left_encoder.getValue()

    def get_right_encoder_value(self):
        """ return right wheel encoder value in radians """
        return self.right_encoder.getValue()


