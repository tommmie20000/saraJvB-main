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
        self.color_stream.start()
        
        # PID control parameters for smooth tracking
        self.pan_kp = 0.05   # Proportional gain for pan (very low to prevent overshoot)
        self.tilt_kp = 0.05  # Proportional gain for tilt (very low to prevent overshoot)
        
        # Current head positions
        self.current_pan = RobotHeadPositions.PAN_MID
        self.current_tilt = RobotHeadPositions.TILT_MID
        
        # Camera center point
        self.center_x = 320  # Half of 640 (typical width)
        self.center_y = 240  # Half of 480 (typical height)
        
        # Deadzone to prevent jittering (larger to reduce oscillation)
        self.deadzone_x = 60
        self.deadzone_y = 60
        
        # Movement smoothing
        self.movement_delay = 0.1  # Delay between movements in seconds
        self.last_move_time = 0
        
        print("Ball Tracker initialized")
    
    def detect_red_ball(self, frame):
        """
        Detects a red ball in the frame and returns its center coordinates.
        Returns (x, y, radius) or (None, None, None) if not found.
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
                return int(x), int(y), int(radius)
        
        return None, None, None
    
    def calculate_head_adjustment(self, ball_x, ball_y):
        """
        Calculate how much to adjust pan and tilt based on ball position.
        Returns (new_pan, new_tilt) positions.
        """
        if ball_x is None or ball_y is None:
            return self.current_pan, self.current_tilt
        
        # Calculate error (distance from center)
        error_x = ball_x - self.center_x
        error_y = ball_y - self.center_y
        
        # Apply deadzone
        if abs(error_x) < self.deadzone_x:
            error_x = 0
        if abs(error_y) < self.deadzone_y:
            error_y = 0
        
        # Calculate adjustments
        pan_adjustment = error_x * self.pan_kp
        tilt_adjustment = -error_y * self.tilt_kp
        
        # Calculate new positions
        new_pan = self.current_pan + pan_adjustment
        new_tilt = self.current_tilt + tilt_adjustment
        
        # Clamp to valid ranges
        new_pan = max(RobotHeadPositions.PAN_LEFT, 
                      min(RobotHeadPositions.PAN_RIGHT, new_pan))
        new_tilt = max(RobotHeadPositions.TILT_DOWN, 
                       min(RobotHeadPositions.TILT_UP, new_tilt))
        
        return new_pan, new_tilt
    
    def move_head(self, pan_pos, tilt_pos):
        """Move the robot head to specified pan and tilt positions."""
        self.robot.head.pan_motor.move(position=int(pan_pos))
        self.robot.head.tilt_motor.move(position=int(tilt_pos))
        self.current_pan = pan_pos
        self.current_tilt = tilt_pos
    
    def run(self):
        """Main tracking loop."""
        print("Starting ball tracking...")
        print("Press 'q' to quit")
        print("Press 'h' to return head to home position")
        
        try:
            while True:
                # Read frame from camera
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
                
                # Detect red ball
                ball_x, ball_y, radius = self.detect_red_ball(frame)
                
                # Draw detection on frame
                if ball_x is not None:
                    # Draw circle around detected ball
                    cv2.circle(frame, (ball_x, ball_y), radius, (0, 255, 0), 2)
                    cv2.circle(frame, (ball_x, ball_y), 5, (0, 0, 255), -1)
                    
                    # Draw line from center to ball
                    cv2.line(frame, (self.center_x, self.center_y), 
                            (ball_x, ball_y), (255, 0, 0), 2)
                    
                    # Calculate new position
                    new_pan, new_tilt = self.calculate_head_adjustment(ball_x, ball_y)
                    
                    # Only move if enough time has passed and position changed significantly
                    current_time = time.time()
                    if (current_time - self.last_move_time >= self.movement_delay and
                        (abs(new_pan - self.current_pan) > 5 or abs(new_tilt - self.current_tilt) > 5)):
                        self.move_head(new_pan, new_tilt)
                        self.last_move_time = current_time
                    
                    # Display info
                    cv2.putText(frame, f"Ball: ({ball_x}, {ball_y})", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, f"Pan: {int(self.current_pan)} Tilt: {int(self.current_tilt)}", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                else:
                    cv2.putText(frame, "No ball detected", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # Draw center crosshair
                cv2.line(frame, (self.center_x - 20, self.center_y), 
                        (self.center_x + 20, self.center_y), (255, 255, 255), 1)
                cv2.line(frame, (self.center_x, self.center_y - 20), 
                        (self.center_x, self.center_y + 20), (255, 255, 255), 1)
                
                # Show frame
                cv2.imshow("Ball Tracking", frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('h'):
                    print("Returning to home position...")
                    self.robot.head.pan_motor.home()
                    self.robot.head.tilt_motor.home()
                    self.current_pan = RobotHeadPositions.PAN_MID
                    self.current_tilt = RobotHeadPositions.TILT_MID
                    time.sleep(2)
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        print("Cleaning up...")
        self.color_stream.stop()
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