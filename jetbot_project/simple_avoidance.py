import time
import math
import signal
import sys

# --- Configuration ---
SAFE_DISTANCE = 0.3    # Meters
DRIVE_SPEED = 0.3      # 0.0 to 1.0
TURN_SPEED = 0.3       # 0.0 to 1.0
LIDAR_PORT = '/dev/ttyACM1'

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

# --- Helper Functions ---
def stop_robot(signal_received, frame):
    """Clean shutdown on Ctrl+C"""
    print("\nStopping robot...")
    robot.stop()
    if 'lidar' in globals() and lidar:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    sys.exit(0)

# Register signal handler
signal.signal(signal.SIGINT, stop_robot)

# --- Main Logic ---
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

# --- Main Logic ---
def main():
    global HAS_LIDAR, SAFE_DISTANCE, DRIVE_SPEED, TURN_SPEED
    lidar = None

    if HAS_LIDAR:
        try:
            lidar = connect_lidar()
        except:
            HAS_LIDAR = False

    print("Starting avoidance loop. Press Ctrl+C to stop.")
    
    # If no Lidar, we just spin or move blindly (Safety: Just stop after a bit)
    if not HAS_LIDAR:
        print("No Lidar active. Moving forward blindly for 2s then stopping.")
        robot.forward(DRIVE_SPEED)
        time.sleep(2)
        robot.stop()
        return

    try:
        while True:
            try:
                # max_buf_meas=500 helps reduce latency
                iterator = lidar.iter_scans(max_buf_meas=500)
                # scan format: [(quality, angle, distance_mm), ...]
                for scan in iterator:
                    # 1. Process Scan
                    min_front_dist = float('inf')
                    
                    for (_, angle, dist_mm) in scan:
                        dist_m = dist_mm / 1000.0
                        
                        # Filter invalid readings
                        if dist_m <= 0:
                            continue

                        # Check Front Sector
                        # Front is 0 degrees. Checking +/- 30 deg (330-360 and 0-30)
                        if angle >= 330 or angle <= 30:
                            if dist_m < min_front_dist:
                                min_front_dist = dist_m

                    # 2. Decide & Act
                    if min_front_dist < SAFE_DISTANCE:
                        print(f"Obstacle detected ({min_front_dist:.2f}m)! Turning...")
                        robot.left(TURN_SPEED)
                    else:
                        # print(f"Path clear ({min_front_dist:.2f}m). Forward.")
                        robot.forward(DRIVE_SPEED)
                    
            except RPLidarException as e:
                print(f"Lidar error: {e}. Reconnecting...")
                robot.stop()
                try:
                    lidar.stop()
                    lidar.stop_motor()
                    lidar.disconnect()
                except:
                    pass
                time.sleep(1)
                try:
                    lidar = connect_lidar()
                except:
                    print("Reconnection failed. Retrying...")
                    time.sleep(1)
            except Exception as e:
                print(f"Runtime error: {e}")
                break

    finally:
        stop_robot(None, None)

if __name__ == "__main__":
    main()

