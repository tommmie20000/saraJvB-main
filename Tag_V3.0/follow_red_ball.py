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
        self.deadzone_x = 15
        self.deadzone_y = 15
        
        # Movement smoothing
        self.smoothing_factor = 0.3  # How quickly to approach target (0-1, lower = smoother)
        self.min_movement = 1  # Minimum movement threshold to prevent micro-adjustments

        # Contour shape filtering (prefer round-ish objects)
        self.min_contour_area = 200
        self.circularity_min = 0.55  # soft threshold for circularity (0..1)
        self.aspect_ratio_min = 0.5  # ellipse aspect ratio (minor/major) to prefer rounder objects

        # Base follow parameters (use ball size instead of depth)
        self.target_radius = 80  # desired ball radius in pixels when at target distance (increased)
        self.radius_deadzone = 5  # pixels (narrower deadzone to allow closer movement)
        self.radius_scale = 1.0  # sensitivity multiplier for pixel error -> speed
        self.max_forward_speed = 30
        self.max_rotation_speed = 20
        self.base_move_enabled = True
        self.last_base_move_time = time.time()
        
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

        best_score = 0
        best_circle = (None, None, None)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_contour_area:
                continue

            peri = cv2.arcLength(cnt, True)
            if peri <= 0:
                continue

            circularity = 4.0 * np.pi * area / (peri * peri)

            # Estimate enclosing circle
            ((x, y), radius) = cv2.minEnclosingCircle(cnt)
            if radius <= 10:
                continue

            # Aspect ratio from fitted ellipse (if possible)
            aspect_score = 1.0
            if len(cnt) >= 5:
                try:
                    (_, _), (MA, ma), _ = cv2.fitEllipse(cnt)
                    if MA > 0 and ma > 0:
                        ar = min(MA, ma) / max(MA, ma)
                        aspect_score = ar if ar > self.aspect_ratio_min else ar * 0.5
                except Exception:
                    aspect_score = 1.0

            # Score combination: prefer larger area and higher circularity and aspect ratio
            # Circularity near 1 is best; use exponent to emphasize roundness
            score = area * (max(circularity, 0.0) ** 1.8) * aspect_score

            if score > best_score:
                best_score = score
                best_circle = (int(x), int(y), int(radius))

        if best_circle[0] is not None:
            return best_circle

        return None, None, None
    
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

    def calculate_base_movement(self, ball_x, radius=None, depth_mm=None):
        """Calculate base movement based on ball size (preferred) or depth (fallback).
        Returns (forward, sideways, rotation).
        """
        if not self.base_move_enabled or ball_x is None:
            return 0, 0, 0

        forward_velocity = 0

        # Prefer radius-based control
        if radius is not None and radius > 0:
            pixel_error = self.target_radius - radius
            if abs(pixel_error) > self.radius_deadzone:
                forward_velocity = int(np.clip(pixel_error * self.radius_scale, -self.max_forward_speed, self.max_forward_speed))
        elif depth_mm is not None and depth_mm != 0:
            # fallback (not used in this file normally)
            distance_error = depth_mm - (self.target_radius * 10)
            if abs(distance_error) > self.radius_deadzone * 10:
                forward_velocity = int(np.clip(distance_error * 0.05, -self.max_forward_speed, self.max_forward_speed))

        # Rotation based on horizontal error
        horizontal_error = ball_x - self.center_x
        rotation_velocity = 0
        if abs(horizontal_error) > self.deadzone_x * 1.5:
            # invert sign: positive horizontal_error (ball to right) -> rotate left (negative),
            # flip if your robot's rotation sign is opposite
            rotation_velocity = int(np.clip(-horizontal_error * 0.1, -self.max_rotation_speed, self.max_rotation_speed))

        return forward_velocity, 0, rotation_velocity

    def move_base(self, forward_velocity, sideways_velocity, rotation_velocity):
        self.robot.base.move(
            Sideways_Velocity=sideways_velocity,
            Forward_Velocity=forward_velocity,
            Rotation_Velocity=rotation_velocity,
        )
        self.last_base_move_time = time.time()
    
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
                    
                    # Display info
                    cv2.putText(frame, f"Ball: ({ball_x}, {ball_y})", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, f"Pan: {int(self.current_pan)} Tilt: {int(self.current_tilt)}", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # Always calculate and move smoothly (even without ball, to settle)
                new_pan, new_tilt = self.calculate_head_adjustment(ball_x, ball_y)
                
                # Move if position changed
                if new_pan != self.current_pan or new_tilt != self.current_tilt:
                    self.move_head(new_pan, new_tilt)
                else:
                    cv2.putText(frame, "No ball detected", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                # Calculate base movement using radius (size)
                forward_vel, sideways_vel, rotation_vel = self.calculate_base_movement(ball_x, radius)
                if forward_vel != 0 or rotation_vel != 0:
                    self.move_base(forward_vel, sideways_vel, rotation_vel)
                else:
                    self.robot.base.move_stop()
                
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
                elif key == ord('b'):
                    self.base_move_enabled = not self.base_move_enabled
                    status = "ENABLED" if self.base_move_enabled else "DISABLED"
                    print(f"Base movement: {status}")
                    if not self.base_move_enabled:
                        self.robot.base.move_stop()
                elif key == ord(']'):
                    # increase desired close distance (larger target radius)
                    self.target_radius += 5
                    print(f"target_radius -> {self.target_radius}")
                elif key == ord('['):
                    self.target_radius = max(5, self.target_radius - 5)
                    print(f"target_radius -> {self.target_radius}")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        print("Cleaning up...")
        self.robot.base.move_stop()
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