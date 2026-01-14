import time
import math
import signal
import sys

# --- Configuration ---
SAFE_DISTANCE = 0.25   # Meters (matches Webots)
DRIVE_SPEED = 0.3      # Duty Cycle (0.0 to 1.0)
TURN_SPEED = 0.4       # Duty Cycle (0.0 to 1.0) - Increased slightly for reliable turns
LIDAR_PORT = '/dev/ttyACM1'

# State Durations (from Webots script)
# Note: Physical robot turn rates vary. 
# Webots model turns at ~1.57 rad/s (90 deg/s).
# If the real robot turns slower, increase DURATION_TURN.
DURATION_TURN = 0.4       # Seconds (approx for 90 deg)
DURATION_SIDE = 1.0       # Seconds
DURATION_PARALLEL = 2.0   # Seconds

# --- Hardware Imports ---
# 1. JetBot Motors
try:
    from jetbot import Robot
    robot = Robot()
    print("JetBot Motors initialized.")
except ImportError:
    print("Error: 'jetbot' library not found. Is this a JetBot?")
    sys.exit(1)

# 2. RPLidar
try:
    from rplidar import RPLidar, RPLidarException
    HAS_LIDAR = True
    print("RPLidar library found.")
except ImportError:
    try:
        from rplidar import RPLidar
        RPLidarException = Exception
        HAS_LIDAR = True
        print("RPLidar library found (generic exception).")
    except ImportError:
        HAS_LIDAR = False
        print("Warning: 'rplidar' library not found. Running in BLIND/DUMMY mode.")

# --- Global State ---
current_state = 'forward'
state_start_time = 0.0
turn_multiplier = -1 # -1 for Right, 1 for Left

# --- Helper Functions ---
def stop_robot(signal_received, frame):
    """Clean shutdown on Ctrl+C"""
    print("\nStopping robot...")
    robot.stop()
    if 'lidar' in globals() and lidar:
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except:
            pass
    sys.exit(0)

# Register signal handler
signal.signal(signal.SIGINT, stop_robot)

def connect_lidar():
    """Connects to the RPLidar and returns the object."""
    try:
        # timeout=3 helps with sync issues
        new_lidar = RPLidar(LIDAR_PORT, timeout=3)
        new_lidar.start_motor()
        time.sleep(1) # Warm up motor
        print(f"Lidar connected on {LIDAR_PORT}")
        return new_lidar
    except Exception as e:
        print(f"Failed to connect to Lidar: {e}")
        raise e

def get_lidar_metrics(scan):
    """
    Process raw scan data to extract sectors.
    RPLidar Scan: [(quality, angle, dist_mm), ...]
    Angle 0 is usually front.
    """
    dist_front = float('inf')
    min_left = float('inf')
    min_right = float('inf')

    # Front cone: +/- 20 degrees (340-360 and 0-20)
    # Left Sector: 20 to 90 degrees
    # Right Sector: 270 to 340 degrees
    
    for (_, angle, dist_mm) in scan:
        dist_m = dist_mm / 1000.0
        if dist_m <= 0: continue

        # Check Front
        if angle > 340 or angle < 20:
            if dist_m < dist_front:
                dist_front = dist_m
        
        # Check Left Sector (20 to 90)
        if 20 < angle < 90:
            if dist_m < min_left:
                min_left = dist_m
        
        # Check Right Sector (270 to 340)
        if 270 < angle < 340:
            if dist_m < min_right:
                min_right = dist_m
                
    return dist_front, min_left, min_right

# --- Main Logic ---
def main():
    global HAS_LIDAR, current_state, state_start_time, turn_multiplier
    lidar = None

    print("Starting Box Avoidance Controller. Press Ctrl+C to stop.")

    while True:
        try:
            if HAS_LIDAR:
                if lidar is None:
                    lidar = connect_lidar()
            else:
                print("No Lidar. Cannot run avoidance logic. Exiting.")
                return

            state_start_time = time.time()

            # Using iter_scans as the main loop driver.
            # This typically runs at 5-10Hz depending on Lidar speed.
            for scan in lidar.iter_scans(max_buf_meas=500):
                current_time = time.time()
                
                # 1. See
                dist_front, min_left, min_right = get_lidar_metrics(scan)

                # 2. Think (State Machine)
                # Default Motor Commands
                move_cmd = 'stop' # forward, left, right, stop

                if current_state == 'forward':
                    move_cmd = 'forward'
                    # Obstacle check
                    if dist_front < SAFE_DISTANCE:
                        print(f"Obstacle detected at {dist_front:.2f}m. Initiating maneuver.")
                        
                        # Determine direction based on side sectors
                        # Note: Webots logic: if min_right < min_left (Right is closer) -> turn_multiplier = -1 (Turn Right)
                        if min_right < min_left:
                            turn_multiplier = -1 # Go Right
                            print(f"Right ({min_right:.2f}) < Left ({min_left:.2f}) -> Turning Right")
                        else:
                            turn_multiplier = 1  # Go Left
                            print(f"Left ({min_left:.2f}) <= Right ({min_right:.2f}) -> Turning Left")
                        
                        current_state = 'avoid_turn_1'
                        state_start_time = current_time

                elif current_state == 'avoid_turn_1':
                    # Turn 90 deg
                    move_cmd = 'left' if turn_multiplier == 1 else 'right'
                    if current_time - state_start_time >= DURATION_TURN:
                        current_state = 'avoid_move_out'
                        state_start_time = current_time

                elif current_state == 'avoid_move_out':
                    # Move away from line
                    move_cmd = 'forward'
                    if current_time - state_start_time >= DURATION_SIDE:
                        current_state = 'avoid_turn_2'
                        state_start_time = current_time

                elif current_state == 'avoid_turn_2':
                    # Turn parallel (-turn_multiplier)
                    # If we turned Right (-1) initially, we now turn Left (1) to be parallel
                    move_cmd = 'right' if turn_multiplier == 1 else 'left'
                    if current_time - state_start_time >= DURATION_TURN:
                        current_state = 'avoid_move_parallel'
                        state_start_time = current_time

                elif current_state == 'avoid_move_parallel':
                    move_cmd = 'forward'
                    if current_time - state_start_time >= DURATION_PARALLEL:
                        current_state = 'avoid_turn_3'
                        state_start_time = current_time

                elif current_state == 'avoid_turn_3':
                    # Turn in (-turn_multiplier)
                    # Same direction as Turn 2 (to cut back in)
                    move_cmd = 'right' if turn_multiplier == 1 else 'left'
                    if current_time - state_start_time >= DURATION_TURN:
                        current_state = 'avoid_move_in'
                        state_start_time = current_time

                elif current_state == 'avoid_move_in':
                    move_cmd = 'forward'
                    if current_time - state_start_time >= DURATION_SIDE:
                        current_state = 'avoid_turn_4'
                        state_start_time = current_time

                elif current_state == 'avoid_turn_4':
                    # Turn back to original heading (turn_multiplier)
                    move_cmd = 'left' if turn_multiplier == 1 else 'right'
                    if current_time - state_start_time >= DURATION_TURN:
                        current_state = 'forward'
                        state_start_time = current_time
                        print("Maneuver complete. Resuming forward.")

                # 3. Act
                if move_cmd == 'forward':
                    robot.forward(DRIVE_SPEED)
                elif move_cmd == 'left':
                    robot.left(TURN_SPEED)
                elif move_cmd == 'right':
                    robot.right(TURN_SPEED)
                else:
                    robot.stop()

        except RPLidarException as e:
            print(f"Lidar error: {e}. Retrying in 1 second...")
            robot.stop()
            if lidar:
                try:
                    lidar.stop()
                    lidar.stop_motor()
                    lidar.disconnect()
                except:
                    pass
            lidar = None
            time.sleep(1)
            continue
        except Exception as e:
            print(f"Runtime error: {e}. Retrying...")
            robot.stop()
            if lidar:
                try:
                    lidar.stop()
                    lidar.stop_motor()
                    lidar.disconnect()
                except:
                    pass
            lidar = None
            time.sleep(1)
            continue

if __name__ == "__main__":
    main()


