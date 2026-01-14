"""turtlebot_wall-following controller"""

# Controller for Turtlebot3 obstacle avoidance behavior.
# Implements a rectangular detour maneuver upon obstacle detection.

from controller import Robot
import math

#-------------------------------------------------------
# Initialize variables

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]

# Physical parameters for the kinematics model (constants)
R = 0.035    # radius of the wheels [m] (TurtleBot3 Burger standard)
D = 0.160    # distance between the wheels [m]

# States
# forward: Move forward until obstacle detected
# avoid_turn_1: Turn 90 deg right
# avoid_move_out: Move forward to clear side
# avoid_turn_2: Turn 90 deg left (parallel to original path)
# avoid_move_parallel: Move forward past the object
# avoid_turn_3: Turn 90 deg left (cut back in front of object)
# avoid_move_in: Move forward to return to original line
# avoid_turn_4: Turn 90 deg right (resume original heading)

current_state = 'forward'
state_start_time = 0.0

# Parameters
SPEED_LINEAR = 0.2       # [m/s]
SPEED_ANGULAR = 1.57     # [rad/s] (approx 90 deg/s)
TURN_ANGLE = math.pi / 2 # 90 degrees in radians

DURATION_TURN = TURN_ANGLE / SPEED_ANGULAR
DURATION_SIDE = 1.5      # Duration to move out/in [s]
DURATION_PARALLEL = 3.5  # Duration to move parallel [s]

#-------------------------------------------------------
# Initialize devices

# 2D-LiDAR
lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()

# motors    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


#######################################################################
# Functions

def wheel_speed_commands(u_d, w_d, D, R):
    """Converts desired speeds to wheel speed commands"""
    wr_d = float((2*u_d + D*w_d)/(2*R))
    wl_d = float((2*u_d - D*w_d)/(2*R))
    return wl_d, wr_d


#######################################################################
# Main loop: See-think-act cycle

while robot.step(timestep) != -1:

    #----------------------------- See -----------------------------------
    current_time = robot.getTime()
    pointCloud = lidar.getRangeImage()
    
    # Detect obstacle in front (using center ray at index 0)
    # Lidar is rotated 180 degrees: index 0 is front.
    if pointCloud:
        dist_front = pointCloud[0]
    else:
        dist_front = float('inf')

    #----------------------------- Think ---------------------------------
    u_d = 0.0
    w_d = 0.0
    
    # State Machine
    if current_state == 'forward':
        u_d = SPEED_LINEAR
        # Obstacle check (0.25m threshold)
        if dist_front < 0.25: 
            current_state = 'avoid_turn_1'
            state_start_time = current_time
            
    elif current_state == 'avoid_turn_1':
        w_d = -SPEED_ANGULAR # Turn Right
        if current_time - state_start_time >= DURATION_TURN:
            current_state = 'avoid_move_out'
            state_start_time = current_time

    elif current_state == 'avoid_move_out':
        u_d = SPEED_LINEAR
        if current_time - state_start_time >= DURATION_SIDE:
            current_state = 'avoid_turn_2'
            state_start_time = current_time

    elif current_state == 'avoid_turn_2':
        w_d = SPEED_ANGULAR # Turn Left (to parallel)
        if current_time - state_start_time >= DURATION_TURN:
            current_state = 'avoid_move_parallel'
            state_start_time = current_time

    elif current_state == 'avoid_move_parallel':
        u_d = SPEED_LINEAR
        if current_time - state_start_time >= DURATION_PARALLEL:
            current_state = 'avoid_turn_3'
            state_start_time = current_time

    elif current_state == 'avoid_turn_3':
        w_d = SPEED_ANGULAR # Turn Left (to face path/in front of object)
        if current_time - state_start_time >= DURATION_TURN:
            current_state = 'avoid_move_in'
            state_start_time = current_time

    elif current_state == 'avoid_move_in':
        u_d = SPEED_LINEAR
        if current_time - state_start_time >= DURATION_SIDE:
            current_state = 'avoid_turn_4'
            state_start_time = current_time

    elif current_state == 'avoid_turn_4':
        w_d = -SPEED_ANGULAR # Turn Right (resume original heading)
        if current_time - state_start_time >= DURATION_TURN:
            current_state = 'forward'
            state_start_time = current_time

    #----------------------------- Act -----------------------------------
    leftSpeed, rightSpeed = wheel_speed_commands(u_d, w_d, D, R)
    
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    # Debug output (optional, uncomment to trace)
    # print(f'State: {current_state}, Dist: {dist_front:.2f}')
