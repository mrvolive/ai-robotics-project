"""Turtlebot controller using Braitenberg obstacle avoidance.

This module implements the main controller for a Turtlebot robot that navigates
using a Braitenberg vehicle logic.
"""

from brain import Brain
from controller import Robot
from encoders import Encoders
from lidar import Lidar
from memory import Memory
from motors import Motors


class TurtlebotWallFollowing:
    """Main controller class for Turtlebot navigation.
    
    This class orchestrates the robot control system. It initializes the
    hardware interfaces and the decision-making brain, then runs the
    main control loop.
    """

    def __init__(self):
        """Initialize the robot controller and all subsystems.

        Creates the Webots Robot instance, initializes the simulation timestep,
        creates the memory system, and initializes all robot devices.
        Also creates the brain for decision making.
        """
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.memory = Memory()

        self._init_devices()
        self.brain = Brain(self.memory, self.lidar, self.encoders)

    def _init_devices(self):
        """Initialize robot devices: Lidar, Motors, and Encoders.

        Retrieves the required devices from the Webots simulation and creates
        the corresponding wrapper instances for each device.
        """
        self.lidar = Lidar(self.robot.getDevice("LDS-01"), self.timestep)
        self.motors = Motors(
            self.robot.getDevice("left wheel motor"),
            self.robot.getDevice("right wheel motor"),
        )
        self.encoders = Encoders(
            self.robot.getDevice("left wheel sensor"),
            self.robot.getDevice("right wheel sensor"),
            self.timestep,
        )

    def run(self):
        """Main control loop.

        Executes the sense-think-act cycle for each simulation step:
        1. Think: Get desired wheel speeds from brain
        2. Act: Set motor velocities
        """
        while self.robot.step(self.timestep) != -1:
            # --- Think ---
            # The brain now returns linear and angular velocities
            linear_vel, angular_vel = self.brain.think()

            # --- Act ---
            self.motors.set_velocity(linear_vel, angular_vel)

            # Optional: Debug output
            # print(f"Cmd: v={linear_vel:.2f}, w={angular_vel:.2f}")


if __name__ == "__main__":
    controller = TurtlebotWallFollowing()
    controller.run()