from jetbot import Robot
from jetbot import Camera
from jetbot import bgr8_to_jpeg
from jetbot import Heartbeat
import numpy as np
import traitlets
import ipywidgets.widgets as widgets
import time
import sys
import select
import tty
import termios
import cv2
from enum import Enum, auto
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
        print("Warning: 'rplidar' library not found. Lidar avoidance disabled.")

# --- Lidar Configuration ---
LIDAR_PORT = '/dev/ttyACM1'
SAFE_DISTANCE = 0.25

class Kinematics:
    """Handle robot kinematics calculations"""
    
    def __init__(self, wheel_radius, wheel_distance, pulses_per_turn):
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        self.pulses_per_turn = pulses_per_turn
    
    def wheel_speed_commands(self, u_desired, w_desired):
        """Converts desired speeds to wheel speed commands"""
        wr_desired = float((2*u_desired + self.wheel_distance*w_desired) / (2*self.wheel_radius))
        wl_desired = float((2*u_desired - self.wheel_distance*w_desired) / (2*self.wheel_radius))
        return wl_desired, wr_desired
    
    def get_angular_and_linear_speed(self, wl, wr):
        """Calculate robot linear and angular speeds from wheel speeds"""
        u = self.wheel_radius / 2.0 * (wr + wl)
        w = self.wheel_radius / self.wheel_distance * (wr - wl)
        return u, w
    
    def get_wheel_speed(self, encoders, old_encoders, delta_t):
        """Calculate wheel speeds from encoder values"""
        ang_diff_l = 2*np.pi*(encoders[0] - old_encoders[0]) / self.pulses_per_turn
        ang_diff_r = 2*np.pi*(encoders[1] - old_encoders[1]) / self.pulses_per_turn
        
        wl = ang_diff_l / delta_t
        wr = ang_diff_r / delta_t
        
        return wl, wr
    
    def get_robot_pose(self, u, w, x_old, y_old, phi_old, delta_t):
        """Calculate robot pose from velocities"""
        delta_phi = w * delta_t
        phi = phi_old + delta_phi
        
        # Normalize angle to [-pi, pi]
        if phi >= np.pi:
            phi = phi - 2*np.pi
        elif phi < -np.pi:
            phi = phi + 2*np.pi
        
        delta_x = u * np.cos(phi) * delta_t
        delta_y = u * np.sin(phi) * delta_t
        x = x_old + delta_x
        y = y_old + delta_y
        
        return x, y, phi

class State(Enum):
    FOLLOWING_LINE = auto()
    RIGHT_DODGE = auto()
    LEFT_DODGE = auto()
    STOP_SIGN = auto()
    TURN_RIGHT_SIGN = auto()
    TURN_LEFT_SIGN = auto()
    SHARP_TURN_LEFT = auto()
    SHARP_TURN_RIGHT = auto()

class RobotController:
    def __init__(self):
        # Hardware Setup
        self.robot = Robot()
        self.camera = Camera.instance()
        self.image_widget = widgets.Image(format='jpeg', width=300, height=300)
        self.camera_link = traitlets.dlink((self.camera, 'value'), (self.image_widget, 'value'), transform=bgr8_to_jpeg)
        
        # Lidar Setup
        self.lidar = None
        if HAS_LIDAR:
            self.lidar = self.connect_lidar()
        
        # Kinematics Setup
        self.WHEEL_RADIUS = 0.325
        self.WHEEL_DISTANCE = 0.15
        self.PULSES_PER_TURN = 330
        self.kinematics = Kinematics(self.WHEEL_RADIUS, self.WHEEL_DISTANCE, self.PULSES_PER_TURN)
        
        # State Machine Setup
        self.current_state = State.FOLLOWING_LINE
        self.state_start_time = time.time()
        self.dodge_phase = 0
        
        # Control Variables
        self.prev_steering = 0.0
        self.alpha = 0.2
        self.base_speed = 0.26
        
        # Heartbeat to stop robot if connection lost
        self.heartbeat = Heartbeat(period=0.5)
        self.heartbeat.observe(self.handle_heartbeat_status, names='status')
        
        print("RobotController initialized. Starting in FOLLOWING_LINE state.")
        self.old_settings = termios.tcgetattr(sys.stdin)

    def handle_heartbeat_status(self, change):
        if change['new'] == Heartbeat.Status.dead:
            self.camera_link.unlink()
            self.robot.stop()

    def connect_lidar(self):
        """Connects to the RPLidar and returns the object."""
        try:
            new_lidar = RPLidar(LIDAR_PORT, timeout=3)
            new_lidar.start_motor()
            time.sleep(1)
            print(f"Lidar connected on {LIDAR_PORT}")
            return new_lidar
        except Exception as e:
            print(f"Failed to connect to Lidar: {e}")
            return None

    def get_lidar_metrics(self, scan):
        """
        Process raw scan data to extract sectors.
        RPLidar Scan: [(quality, angle, dist_mm), ...]
        Angle 0 is usually front.
        Returns: (dist_front, min_left, min_right)
        """
        dist_front = float('inf')
        min_left = float('inf')
        min_right = float('inf')

        for (_, angle, dist_mm) in scan:
            dist_m = dist_mm / 1000.0
            if dist_m <= 0: continue

            if angle > 340 or angle < 20:
                if dist_m < dist_front:
                    dist_front = dist_m
            
            if 20 < angle < 90:
                if dist_m < min_left:
                    min_left = dist_m
            
            if 270 < angle < 340:
                if dist_m < min_right:
                    min_right = dist_m
                
        return dist_front, min_left, min_right

    def preprocess(self, frame):
        frame = cv2.medianBlur(frame, 3)
        frame = cv2.addWeighted(frame, 1, np.zeros(frame.shape, frame.dtype), 0, 2)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return frame

    def get_two_line_centers(self, lines, img_width):
        if lines is None or len(lines) == 0:
            return None, None
        
        midpoints = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            mx = (x1 + x2) / 2
            my = (y1 + y2) / 2
            midpoints.append((mx, my))
        
        center_x = img_width / 2
        left_points = [p for p in midpoints if p[0] < center_x]
        right_points = [p for p in midpoints if p[0] >= center_x]
        
        left_center = None
        right_center = None
        
        if left_points:
            left_x = np.mean([p[0] for p in left_points])
            left_y = np.mean([p[1] for p in left_points])
            left_center = (int(left_x), int(left_y))
        
        if right_points:
            right_x = np.mean([p[0] for p in right_points])
            right_y = np.mean([p[1] for p in right_points])
            right_center = (int(right_x), int(right_y))
        
        return left_center, right_center

    def calculate_steering(self, left_center, right_center, img_width, k=1.0):
        if left_center is not None and right_center is not None:
            lane_center_x = (left_center[0] + right_center[0]) / 2
            lane_center_y = (left_center[1] + right_center[1]) / 2
            lane_center = (int(lane_center_x), int(lane_center_y))
        elif left_center is not None:
            estimated_lane_width = img_width * 0.4
            lane_center_x = left_center[0] + estimated_lane_width / 2
            lane_center_y = left_center[1]
            lane_center = (int(lane_center_x), int(lane_center_y))
        elif right_center is not None:
            estimated_lane_width = img_width * 0.4
            lane_center_x = right_center[0] - estimated_lane_width / 2
            lane_center_y = right_center[1]
            lane_center = (int(lane_center_x), int(lane_center_y))
        else:
            return 0.0, None
        
        image_center_x = img_width / 2
        error_x = lane_center[0] - image_center_x
        max_offset = img_width / 2
        normalized_error = error_x / max_offset
        steering = k * normalized_error
        steering = max(-1.0, min(1.0, steering))
        
        return steering, lane_center

    def transition_to(self, new_state):
        if self.current_state != new_state:
            print(f"Transitioning from {self.current_state.name} to {new_state.name}")
            self.current_state = new_state
            self.state_start_time = time.time()
            self.dodge_phase = 0 # Reset dodge phase on transition

    def check_triggers(self):
        """
        Check for conditions to switch states.
        Checks lidar for obstacles and triggers dodge maneuver if needed.
        """
        if HAS_LIDAR and self.lidar is not None and self.current_state == State.FOLLOWING_LINE:
            try:
                scan = next(self.lidar.iter_scans(max_buf_meas=500, min_len=1))
                dist_front, min_left, min_right = self.get_lidar_metrics(scan)
                
                if dist_front < SAFE_DISTANCE:
                    print(f"Obstacle detected at {dist_front:.2f}m. Initiating dodge.")
                    if min_right < min_left:
                        print(f"Right ({min_right:.2f}) < Left ({min_left:.2f}) -> RIGHT_DODGE")
                        self.transition_to(State.RIGHT_DODGE)
                    else:
                        print(f"Left ({min_left:.2f}) <= Right ({min_right:.2f}) -> LEFT_DODGE")
                        self.transition_to(State.LEFT_DODGE)
            except (RPLidarException, StopIteration, Exception) as e:
                pass

    def run_following_line(self):
        # 1. Get Image
        if self.camera.value is None:
            return
            
        image_aux = self.preprocess(self.camera.value)
        h, w = self.camera.value.shape[:2]
        sliced_image = self.camera.value[int(h*0.5):h, :, :]
        
        CANNY_THRESHOLD = 70
        edges = cv2.Canny(image_aux, CANNY_THRESHOLD, CANNY_THRESHOLD * 1.1)
        h_edge, w_edge = edges.shape[:2]
        edges = edges[int(h_edge*0.5):h_edge, :]
        
        lines = cv2.HoughLinesP(
            edges, 1, np.pi/180, 50, minLineLength=10, maxLineGap=50
        )
        
        steering = 0.0
        steering_limited = 0.0
        
        if lines is not None:
            left_center, right_center = self.get_two_line_centers(lines, sliced_image.shape[1])
            steering, _ = self.calculate_steering(
                left_center, right_center, sliced_image.shape[1], k=2
            )
            
            # Smoothing
            self.prev_steering = (1 - self.alpha) * self.prev_steering + self.alpha * steering
            steering_limited = max(-0.5, min(0.5, self.prev_steering))
            
        # Drive
        left_speed, right_speed = self.kinematics.wheel_speed_commands(self.base_speed, steering_limited)
        self.robot.set_motors(left_speed, right_speed)
        
        # Check for transitions
        self.check_triggers()

    def run_dodge(self, direction='right'):
        """
        Blind dodge maneuver: Turn out, Move, Turn parallel, Move, Turn back in, Move, Turn straight.
        direction: 'right' or 'left' (initial turn direction)
        """
        elapsed = time.time() - self.state_start_time
        
        # Parameters (tuned for open loop)
        TURN_DURATION = 0.4
        MOVE_OUT_DURATION = 1.0
        MOVE_PARALLEL_DURATION = 2.0
        
        # Directions
        turn_out = -1 if direction == 'right' else 1
        turn_in = 1 if direction == 'right' else -1
        
        turn_speed = 0.4
        drive_speed = 0.3
        
        # Phases based on time
        # Phase 0: Turn Out
        if elapsed < TURN_DURATION:
            if self.dodge_phase != 0: print(f"Dodge {direction}: Turning Out")
            self.dodge_phase = 0
            # Turn: left motor calls, right motor calls
            # turn_out=-1 (right) -> left wheel > right wheel? 
            # jetbot set_motors(left, right). 
            # Turn right: left=+, right=-
            if direction == 'right':
                self.robot.set_motors(turn_speed, -turn_speed)
            else:
                self.robot.set_motors(-turn_speed, turn_speed)
                
        # Phase 1: Move Out
        elif elapsed < (TURN_DURATION + MOVE_OUT_DURATION):
            if self.dodge_phase != 1: print(f"Dodge {direction}: Moving Out")
            self.dodge_phase = 1
            self.robot.set_motors(drive_speed, drive_speed)
            
        # Phase 2: Turn Parallel (Counter steer)
        elif elapsed < (2*TURN_DURATION + MOVE_OUT_DURATION):
            if self.dodge_phase != 2: print(f"Dodge {direction}: Turning Parallel")
            self.dodge_phase = 2
            if direction == 'right': # To align parallel, turn Left
                self.robot.set_motors(-turn_speed, turn_speed)
            else: # Turn Right
                self.robot.set_motors(turn_speed, -turn_speed)

        # Phase 3: Move Parallel
        elif elapsed < (2*TURN_DURATION + MOVE_OUT_DURATION + MOVE_PARALLEL_DURATION):
            if self.dodge_phase != 3: print(f"Dodge {direction}: Moving Parallel")
            self.dodge_phase = 3
            self.robot.set_motors(drive_speed, drive_speed)
            
        # Phase 4: Turn Back In
        elif elapsed < (3*TURN_DURATION + MOVE_OUT_DURATION + MOVE_PARALLEL_DURATION):
            if self.dodge_phase != 4: print(f"Dodge {direction}: Turning In")
            self.dodge_phase = 4
            if direction == 'right': # Turn Left to go back to line
                self.robot.set_motors(-turn_speed, turn_speed)
            else: # Turn Right
                self.robot.set_motors(turn_speed, -turn_speed)

        # Phase 5: Move In
        elif elapsed < (3*TURN_DURATION + 2*MOVE_OUT_DURATION + MOVE_PARALLEL_DURATION):
            if self.dodge_phase != 5: print(f"Dodge {direction}: Moving In")
            self.dodge_phase = 5
            self.robot.set_motors(drive_speed, drive_speed)
            
        # Phase 6: Turn Straight (Align with line)
        elif elapsed < (4*TURN_DURATION + 2*MOVE_OUT_DURATION + MOVE_PARALLEL_DURATION):
            if self.dodge_phase != 6: print(f"Dodge {direction}: Aligning")
            self.dodge_phase = 6
            if direction == 'right': # Turn Right to face forward
                self.robot.set_motors(turn_speed, -turn_speed)
            else: # Turn Left
                self.robot.set_motors(-turn_speed, turn_speed)
                
        else:
            # Maneuver done
            print(f"Dodge {direction} complete. Returning to Line Following.")
            self.transition_to(State.FOLLOWING_LINE)

    def run_stop_sign(self):
        # Stop for 2 seconds
        if time.time() - self.state_start_time < 2.0:
            self.robot.stop()
        else:
            self.transition_to(State.FOLLOWING_LINE)

    def run_turn_sign(self, direction='right'):
        # Turn 90 degrees (approx 0.5s at speed?)
        if time.time() - self.state_start_time < 0.5:
            if direction == 'right':
                self.robot.set_motors(0.3, -0.3)
            else:
                self.robot.set_motors(-0.3, 0.3)
        else:
            self.transition_to(State.FOLLOWING_LINE)
            
    def run_sharp_turn(self, direction='left'):
        # Sharp turn while moving? Or pivot? 
        # Let's assume a tighter turn radius for a short duration
        if time.time() - self.state_start_time < 1.0:
            if direction == 'left':
                self.robot.set_motors(0.1, 0.4) # Pivot left
            else:
                self.robot.set_motors(0.4, 0.1) # Pivot right
        else:
            self.transition_to(State.FOLLOWING_LINE)

    def update(self):
        if self.current_state == State.FOLLOWING_LINE:
            self.run_following_line()
            
        elif self.current_state == State.RIGHT_DODGE:
            self.run_dodge(direction='right')
            
        elif self.current_state == State.LEFT_DODGE:
            self.run_dodge(direction='left')
            
        elif self.current_state == State.STOP_SIGN:
            self.run_stop_sign()
            
        elif self.current_state == State.TURN_RIGHT_SIGN:
            self.run_turn_sign(direction='right')
            
        elif self.current_state == State.TURN_LEFT_SIGN:
            self.run_turn_sign(direction='left')
            
        elif self.current_state == State.SHARP_TURN_LEFT:
            self.run_sharp_turn(direction='left')
            
        elif self.current_state == State.SHARP_TURN_RIGHT:
            self.run_sharp_turn(direction='right')

    def handle_keys(self):
        if self.camera.value is None:
            return

        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            
            if key == 'q':
                raise KeyboardInterrupt
            elif key == '1':
                self.transition_to(State.FOLLOWING_LINE)
            elif key == '2':
                self.transition_to(State.RIGHT_DODGE)
            elif key == '3':
                self.transition_to(State.LEFT_DODGE)
            elif key == '4':
                self.transition_to(State.STOP_SIGN)
            elif key == '5':
                self.transition_to(State.TURN_RIGHT_SIGN)
            elif key == '6':
                self.transition_to(State.TURN_LEFT_SIGN)
            elif key == '7':
                self.transition_to(State.SHARP_TURN_LEFT)
            elif key == '8':
                self.transition_to(State.SHARP_TURN_RIGHT)

    def start(self):
        print("Starting Control Loop.")
        print("Debug Keys: 1:Line, 2:R-Dodge, 3:L-Dodge, 4:Stop, 5:R-Turn, 6:L-Turn, 7:Sharp-L, 8:Sharp-R, q:Quit")
        tty.setraw(sys.stdin.fileno())
        try:
            while True:
                self.handle_keys()
                self.update()
                # time.sleep(0.01) # Small sleep to prevent 100% CPU usage if needed
        except KeyboardInterrupt:
            self.robot.stop()
            self.camera_link.unlink()
            self.camera.stop()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            if self.lidar is not None:
                try:
                    self.lidar.stop()
                    self.lidar.stop_motor()
                    self.lidar.disconnect()
                except:
                    pass
            
controller = RobotController()         
controller.start()

