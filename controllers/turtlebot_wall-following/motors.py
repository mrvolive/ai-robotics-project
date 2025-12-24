class Motors:
    # Robot Physical Parameters
    R = 0.330  # radius of the wheels [m]
    D = 0.160  # distance between the wheels [m]

    def __init__(self, left_motor, right_motor):
        self.left_motor = left_motor
        self.right_motor = right_motor
        
        # Initialize motors for velocity control
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def set_velocity(self, u_d, w_d):
        """
        Converts desired linear (u_d) and angular (w_d) speeds to wheel speeds.
        u_d: linear velocity (m/s)
        w_d: angular velocity (rad/s)
        """
        wr_d = (2 * u_d + self.D * w_d) / (2 * self.R)
        wl_d = (2 * u_d - self.D * w_d) / (2 * self.R)
        
        self.left_motor.setVelocity(float(wl_d))
        self.right_motor.setVelocity(float(wr_d))
