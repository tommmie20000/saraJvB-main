import sys
import os
import time
import cv2
import numpy as np
from openni import openni2

from Common.sara_library import SaraRobot
from Common.sara_common import RobotHeadPositions

class BallTracker:
    def __init__(self, robot):
        self.robot = robot
        
        # Initialize OpenNI2 - adjust path to your system
        openni2.initialize("/home/thom/saraJvB-main/OpenCV/CameraUbuntu/AstraSDK-v2.1.3/lib/Plugins/openni2/")
        
        # Open camera device
        self.dev = openni2.Device.open_any()
        
        # Create color stream
        self.color_stream = self.dev.create_color_stream()
        self.color_stream.set_video_mode(openni2.VideoMode(pixelFormat=openni2.PIXEL_FORMAT_RGB888, resolutionX=640, resolutionY=480, fps=30))
        self.color_stream.start()
        
        # Create depth stream
        self.depth_stream = self.dev.create_depth_stream()
        self.depth_stream.set_video_mode(openni2.VideoMode(pixelFormat=openni2.PIXEL_FORMAT_DEPTH_1_MM, resolutionX=640, resolutionY=480, fps=30))
        self.depth_stream.start()
        
        # Sync streams
        self.dev.set_depth_color_sync_enabled(True)
        
        # PID control parameters for smooth tracking
        self.pan_kp = 0.08   # Proportional gain for pan
        self.tilt_kp = 0.08  # Proportional gain for tilt
        
        # Current head positions
        self.current_pan = RobotHeadPositions.PAN_MID
        self.current_tilt = RobotHeadPositions.TILT_MID
        
        # Target positions (for smooth interpolation)
        self.target_pan = RobotHeadPositions.PAN_MID
        self.target_tilt = RobotHeadPositions.TILT_MID
        
        # Camera center point
        self.center_x = 320  # Half of 640 (typical width)
        self.center_y = 240  # Half of 480 (typical height)
        
        # Deadzone to prevent jittering (larger to reduce oscillation)
        self.deadzone_x = 60
        self.deadzone_y = 60
        
        # Movement smoothing
        self.smoothing_factor = 0.3  # How quickly to approach target (0-1, lower = smoother)
        self.min_movement = 1  # Minimum movement threshold to prevent micro-adjustments
        
        # Frame skip for depth reading (read depth every N frames for performance)
        self.depth_frame_skip = 2  # Read depth every 2 frames
        self.frame_counter = 0
        self.last_depth_mm = None
        
        # Base movement parameters (depth-based following)
        self.target_distance = 800  # Target distance in mm (80cm)
        self.distance_deadzone = 150  # Don't move if within ±15cm of target
        self.max_forward_speed = 30  # Maximum forward/backward speed
        self.max_rotation_speed = 20  # Maximum rotation speed when centering
        self.base_move_enabled = True  # Toggle for base movement
        
        # Last base movement time (for safety timeout)
        self.last_base_move_time = time.time()
        
        print("Ball Tracker initialized")
    
    def detect_red_ball(self, frame, depth_frame=None):
        """
        Detects a red ball in the frame and returns its center coordinates and depth.
        Returns (x, y, radius, depth_mm) or (None, None, None, None) if not found.
        """
        # Convert BGR to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define range for red color
        # Red wraps around in HSV, so we need two ranges
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # Find the largest contour (assume it's the ball)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get the minimum enclosing circle
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            
            # Only return if the radius is large enough (filter noise)
            if radius > 10:
                depth_mm = None
                if depth_frame is not None:
                    # Get depth at ball center
                    try:
                        depth_mm = depth_frame[int(y), int(x)]
                    except:
                        depth_mm = None
                
                return int(x), int(y), int(radius), depth_mm
        
        return None, None, None, None
    
    def calculate_head_adjustment(self, ball_x, ball_y):
        """
        Calculate how much to adjust pan and tilt based on ball position.
        Updates target position, returns smoothed current position.
        """
        if ball_x is None or ball_y is None:
            # No ball detected, maintain current position
            return self.current_pan, self.current_tilt
        
        # Calculate error (distance from center)
        error_x = ball_x - self.center_x
        error_y = ball_y - self.center_y
        
        # Apply deadzone
        if abs(error_x) < self.deadzone_x:
            error_x = 0
        if abs(error_y) < self.deadzone_y:
            error_y = 0
        
        # Calculate target adjustments
        pan_adjustment = error_x * self.pan_kp
        tilt_adjustment = -error_y * self.tilt_kp
        
        # Update target positions
        self.target_pan = self.current_pan + pan_adjustment
        self.target_tilt = self.current_tilt + tilt_adjustment
        
        # Clamp targets to valid ranges
        self.target_pan = max(RobotHeadPositions.PAN_LEFT, 
                             min(RobotHeadPositions.PAN_RIGHT, self.target_pan))
        self.target_tilt = max(RobotHeadPositions.TILT_DOWN, 
                              min(RobotHeadPositions.TILT_UP, self.target_tilt))
        
        # Smoothly interpolate current position toward target
        pan_diff = self.target_pan - self.current_pan
        tilt_diff = self.target_tilt - self.current_tilt
        
        # Apply smoothing (exponential moving average)
        new_pan = self.current_pan + (pan_diff * self.smoothing_factor)
        new_tilt = self.current_tilt + (tilt_diff * self.smoothing_factor)
        
        # Only update if movement is significant enough
        if abs(new_pan - self.current_pan) < self.min_movement:
            new_pan = self.current_pan
        if abs(new_tilt - self.current_tilt) < self.min_movement:
            new_tilt = self.current_tilt
        
        return new_pan, new_tilt
    
    def move_head(self, pan_pos, tilt_pos):
        """Move the robot head to specified pan and tilt positions."""
        self.robot.head.pan_motor.move(position=int(pan_pos))
        self.robot.head.tilt_motor.move(position=int(tilt_pos))
        self.current_pan = pan_pos
        self.current_tilt = tilt_pos
    
    def calculate_base_movement(self, ball_x, depth_mm):
        """
        Calculate base movement (forward/backward and rotation) based on ball position and depth.
        Returns (forward_velocity, sideways_velocity, rotation_velocity).
        """
        if not self.base_move_enabled or ball_x is None or depth_mm is None or depth_mm == 0:
            return 0, 0, 0
        
        # Calculate forward/backward movement based on depth
        distance_error = depth_mm - self.target_distance
        
        forward_velocity = 0
        if abs(distance_error) > self.distance_deadzone:
            # Positive error = too far, move forward
            # Negative error = too close, move backward
            forward_velocity = int(np.clip(distance_error * 0.05, -self.max_forward_speed, self.max_forward_speed))
        
        # Calculate rotation based on horizontal position
        horizontal_error = ball_x - self.center_x
        
        rotation_velocity = 0
        # Only rotate if ball is significantly off-center (outside head deadzone)
        if abs(horizontal_error) > self.deadzone_x * 1.5:
            # Positive error = ball on right, rotate right (clockwise)
            rotation_velocity = int(np.clip(horizontal_error * 0.1, -self.max_rotation_speed, self.max_rotation_speed))
        
        return forward_velocity, 0, rotation_velocity  # sideways always 0 for now
    
    def move_base(self, forward_velocity, sideways_velocity, rotation_velocity):
        """
        Move the robot base with the given velocities.
        Implements safety timeout - must be called regularly.
        """
        current_time = time.time()
        
        # Move the base
        self.robot.base.move(
            Sideways_Velocity=sideways_velocity,
            Forward_Velocity=forward_velocity,
            Rotation_Velocity=rotation_velocity
        )
        
        self.last_base_move_time = current_time
    
    def run(self):
        """Main tracking loop."""
        print("Starting ball tracking...")
        print("Press 'q' to quit")
        print("Press 'h' to return head to home position")
        print("Press 'b' to toggle base movement on/off")
        print(f"Base movement: {'ENABLED' if self.base_move_enabled else 'DISABLED'}")
        
        try:
            while True:
                self.frame_counter += 1
                
                # Read frame from color camera
                color_frame = self.color_stream.read_frame()
                color_data = color_frame.get_buffer_as_uint8()
                
                # Convert to numpy array
                color_array = np.ctypeslib.as_array(color_data)
                frame = cv2.cvtColor(
                    color_array.reshape((color_frame.height, color_frame.width, 3)),
                    cv2.COLOR_RGB2BGR
                )
                
                # Flip horizontally for mirror effect
                frame = cv2.flip(frame, 1)
                
                # Read depth frame only every N frames for performance
                depth_frame = None
                if self.frame_counter % self.depth_frame_skip == 0:
                    try:
                        depth_frame_raw = self.depth_stream.read_frame()
                        depth_data = depth_frame_raw.get_buffer_as_uint16()
                        depth_array = np.ctypeslib.as_array(depth_data)
                        depth_frame = depth_array.reshape((depth_frame_raw.height, depth_frame_raw.width))
                        depth_frame = cv2.flip(depth_frame, 1)
                    except:
                        depth_frame = None
                
                # Detect red ball with depth
                ball_x, ball_y, radius, depth_mm = self.detect_red_ball(frame, depth_frame)
                
                # Cache depth value if we got a new one
                if depth_mm is not None:
                    self.last_depth_mm = depth_mm
                elif ball_x is not None:
                    # Use last known depth if we have ball but no new depth reading
                    depth_mm = self.last_depth_mm
                
                # Draw detection on frame
                if ball_x is not None:
                    # Draw circle around detected ball
                    cv2.circle(frame, (ball_x, ball_y), radius, (0, 255, 0), 2)
                    cv2.circle(frame, (ball_x, ball_y), 5, (0, 0, 255), -1)
                    
                    # Draw line from center to ball
                    cv2.line(frame, (self.center_x, self.center_y), 
                            (ball_x, ball_y), (255, 0, 0), 2)
                    
                    # Display info
                    info_text = f"Ball: ({ball_x}, {ball_y})"
                    if depth_mm and depth_mm > 0:
                        info_text += f" | Depth: {depth_mm}mm ({depth_mm/10:.1f}cm)"
                    cv2.putText(frame, info_text, (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    cv2.putText(frame, f"Pan: {int(self.current_pan)} Tilt: {int(self.current_tilt)}", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    
                    # Show base movement status
                    base_status = "Base: ENABLED" if self.base_move_enabled else "Base: DISABLED"
                    cv2.putText(frame, base_status, (10, 90),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                else:
                    cv2.putText(frame, "No ball detected", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    base_status = "Base: ENABLED" if self.base_move_enabled else "Base: DISABLED"
                    cv2.putText(frame, base_status, (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                # Always calculate and move head smoothly
                new_pan, new_tilt = self.calculate_head_adjustment(ball_x, ball_y)
                
                # Move head if position changed
                if new_pan != self.current_pan or new_tilt != self.current_tilt:
                    self.move_head(new_pan, new_tilt)
                
                # Calculate and execute base movement
                forward_vel, sideways_vel, rotation_vel = self.calculate_base_movement(ball_x, depth_mm)
                
                if forward_vel != 0 or rotation_vel != 0:
                    self.move_base(forward_vel, sideways_vel, rotation_vel)
                else:
                    # Stop base if no movement needed
                    self.robot.base.move_stop()
                
                # Draw center crosshair
                cv2.line(frame, (self.center_x - 20, self.center_y), 
                        (self.center_x + 20, self.center_y), (255, 255, 255), 1)
                cv2.line(frame, (self.center_x, self.center_y - 20), 
                        (self.center_x, self.center_y + 20), (255, 255, 255), 1)
                
                # Show frame
                cv2.imshow("Ball Tracking", frame)
                
                # Handle keyboard input (waitKey=1 for minimal delay)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('h'):
                    print("Returning to home position...")
                    self.robot.base.move_stop()
                    self.robot.head.pan_motor.home()
                    self.robot.head.tilt_motor.home()
                    self.current_pan = RobotHeadPositions.PAN_MID
                    self.current_tilt = RobotHeadPositions.TILT_MID
                    time.sleep(2)
                elif key == ord('b'):
                    self.base_move_enabled = not self.base_move_enabled
                    status = "ENABLED" if self.base_move_enabled else "DISABLED"
                    print(f"Base movement: {status}")
                    if not self.base_move_enabled:
                        self.robot.base.move_stop()
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        print("Cleaning up...")
        self.robot.base.move_stop()
        self.color_stream.stop()
        self.depth_stream.stop()
        openni2.unload()
        cv2.destroyAllWindows()


def main():
    # Initialize robot
    robot = SaraRobot(logging=False)
    
    time.sleep(1)
    
    # Get versions
    robot.head.getversion()
    robot.body.getversion()
    
    time.sleep(1)
    
    # Home the head
    print("Homing head...")
    robot.head.pan_motor.home()
    robot.head.tilt_motor.home()
    
    time.sleep(3)
    
    # Create and run ball tracker
    tracker = BallTracker(robot)
    tracker.run()
    
    # Stop robot
    robot.stop()
    
    return


if __name__ == "__main__":
    main()