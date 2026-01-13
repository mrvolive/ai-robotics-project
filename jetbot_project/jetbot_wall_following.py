import time
from brain import Brain
from jetbot import Robot
from encoders import Encoders
from lidar import Lidar
from memory import Memory
from motors import Motors

class JetbotWallFollowing:
    """Main controller class for Jetbot navigation.
    
    This class orchestrates the robot control system. It initializes the
    hardware interfaces and the decision-making brain, then runs the
    main control loop.
    """

    def __init__(self):
        """Initialize the robot controller and all subsystems."""
        self.robot = Robot()
        # JetBot doesn't have a 'timestep' like Webots, we define a control loop period
        self.period = 0.1 # 100ms = 10Hz

        self.memory = Memory()

        self._init_devices()
        self.brain = Brain(self.memory, self.lidar, self.encoders)

    def _init_devices(self):
        """Initialize robot devices: Lidar, Motors, and Encoders."""
        # Lidar is initialized (might be dummy if no hardware)
        self.lidar = Lidar()
        
        # Motors are initialized with the JetBot Robot instance
        self.motors = Motors(self.robot)
        
        # Encoders are initialized (stub for JetBot usually)
        self.encoders = Encoders()

    def run(self):
        """Main control loop.

        Executes the sense-think-act cycle.
        """
        print("Starting Jetbot Wall Following Control Loop...")
        try:
            while True:
                start_time = time.time()
                
                # --- Think ---
                # The brain returns linear (m/s) and angular (rad/s) velocities
                linear_vel, angular_vel = self.brain.think()

                # --- Act ---
                self.motors.set_velocity(linear_vel, angular_vel)

                # Optional: Debug output
                # print(f"Cmd: v={linear_vel:.2f}, w={angular_vel:.2f}")
                
                # Sleep to maintain loop rate
                elapsed = time.time() - start_time
                if elapsed < self.period:
                    time.sleep(self.period - elapsed)
                    
        except KeyboardInterrupt:
            print("Stopping...")
            self.motors.stop()

if __name__ == "__main__":
    controller = JetbotWallFollowing()
    controller.run()
