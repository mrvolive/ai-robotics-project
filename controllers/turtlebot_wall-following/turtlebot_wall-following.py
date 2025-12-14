"""turtlebot_wall-following controller"""

# You can us this template to implement the behaviors and maze sover state-machine
# for the Turtlebot3.

import numpy as np
from controller import Robot  # , DistanceSensor, Motor, Compass

# -------------------------------------------------------
# Initialize variables

MAX_SPEED = 40  # 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())  # [ms]
delta_t = robot.getBasicTimeStep() / 1000.0  # [s]

# states
states = ["forward", "turn", "backward"]
current_state = states[0]

# counter: used to maintain an active state for a number of cycles
counter = 0
COUNTER_MAX = 181

# Robot wheel speeds
wl = 0.0  # angular speed of the left wheel [rad/s]
wr = 0.0  # angular speed of the right wheel [rad/s]

# Robot linear and angular speeds
u = 0.0  # linear speed [m/s]
w = 0.0  # angular speed [rad/s]

# Physical parameters for the kinematics model (constants)
R = 0.330  # radius of the wheels [m]
D = 0.160  # distance between the wheels [m]

# Controller gains - define how the reaction of the robot will be:
# higher controller gain will result in faster reaction, but it can cause oscillations
k_1 = 1
k_2 = 1

# -------------------------------------------------------
# Initialize devices

# 2D-LiDAR
lidar = robot.getDevice("LDS-01")
lidar.enable(timestep)
lidar.enablePointCloud()
pointCloud = []  # list to store the point cloud data

# encoders
encoder = []
encoderNames = ["left wheel sensor", "right wheel sensor"]  # defined in the robot model
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)

oldEncoderValues = []

# motors
leftMotor = robot.getDevice("left wheel motor")  # name defined in the robot model
rightMotor = robot.getDevice("right wheel motor")  # name defined in the robot model
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


#######################################################################
# Functions
# write here any function you may need: wheel speed computation, robot speed computation,
# wall-following behavior, etc.


def wheel_speed_commands(u_d, w_d, D, R):
    """Converts desired speeds to wheel speed commands
    Inputs:
        u_d = desired linear speed for the robot [m/s]
        w_d = desired angular speed for the robot [rad/s]
        R = radius of the robot wheel [m]
        D = distance between the left and right wheels [m]
    Returns:
        wr_d = desired speed for the right wheel [rad/s]
        wl_d = desired speed for the left wheel [rad/s]
    """
    wr_d = float((2 * u_d + D * w_d) / (2 * R))
    wl_d = float((2 * u_d - D * w_d) / (2 * R))

    return wl_d, wr_d


#######################################################################
# Main loop: See-think-act cycle
# Perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # ----------------------------- See -----------------------------------
    # Read the LiDAR data
    pointCloud = lidar.getRangeImage()  # [m], 360 values, one per degree

    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())  # [rad]

    # Update old encoder values if not done before
    if len(oldEncoderValues) < 2:
        for i in range(2):
            oldEncoderValues.append(encoder[i].getValue())

    # ----------------------------- Think ---------------------------------
    # Implement the finite-state machine to select the robot behavior
    # Example:
    if current_state == "forward":
        u_d = 2.0
        w_d = 0.0

    if current_state == "left":
        u_d = 0.0
        w_d = 6.0

    if current_state == "right":
        u_d = 0.0
        w_d = -6.0

    if current_state == "backward":
        u_d = -2.0
        w_d = 0.0

    # Transitions:
    if current_state == "forward":
        if pointCloud[180] < 0.2:
            if pointCloud[90] < 0.3:
                current_state = "right"
            else:
                current_state = "left"
            counter = 91 # higher initial value for a shorter curve
        # else:
        #     if pointCloud[90] < 0.15:
        #         current_state = 'left'
        #     if pointCloud[270] < 0.15:
        #         current_state = 'right'
        #     counter = 95

    # Correction de direction ?
    # if current_state == "forward":
    #     print(
    #         f"distances = {pointCloud[180]:.2f}, fright = {pointCloud[179]:.2f}, fleft = {pointCloud[181]:.2f}"
    #     )
    #     offset = pointCloud[181] - pointCloud[179]
    #     print(offset)
    #     if offset > 0.1:
    #         current_state = "left"
    #         counter = 90  # higher initial value for a shorter curve
    #     if offset < -0.1:
    #         current_state = "right"
    #         counter = 90  # higher initial value for a shorter curve

    if current_state == "left" or current_state == "right":
        if counter > COUNTER_MAX:
            current_state = "forward"
    # increment counter
    counter += 1

    # update old encoder values for the next cycle
    oldEncoderValues = encoderValues

    # ----------------------------- Act -----------------------------------
    # Set motor speeds with the values defined by the state-machine
    leftSpeed, rightSpeed = wheel_speed_commands(u_d, w_d, D, R)
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    # Debug
    print(
        f"Current state = {current_state}, distances = {pointCloud[180]:.2f}, fright = {pointCloud[270]:.2f}, fleft = {pointCloud[90]:.2f}, u_d = {u_d:.2f}, w_d = {w_d:.2f}"
    )

    # Repeat all steps while the simulation is running.
