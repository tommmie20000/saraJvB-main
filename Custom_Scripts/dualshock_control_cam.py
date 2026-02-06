import time
import pygame
import threading
from Common.sara_library import SaraRobot, ColorLed

# Try to import camera libraries, make them optional
CAMERA_AVAILABLE = False
try:
    from openni import openni2
    import numpy as np
    import cv2
    CAMERA_AVAILABLE = True
except (ImportError, TypeError) as e:
    print(f"Camera libraries not available: {e}")
    print("Running without camera support.")


DEADZONE = 0.15
TRIGGER_DEADZONE = 0.05


def apply_deadzone(value, dz=DEADZONE):
    return 0 if abs(value) < dz else value


def apply_trigger_deadzone(value, dz=TRIGGER_DEADZONE):
    return 0 if value < dz else value


def set_police_leds(robot, color_left, color_right):
    robot.left_arm.led.setcolor(color=color_left, blink=ColorLed.LED_ON)
    robot.right_arm.led.setcolor(color=color_right, blink=ColorLed.LED_ON)
    robot.base.led.setcolor(color=color_left, blink=ColorLed.LED_ON)
    robot.head.left_led.setcolor(color=color_left, blink=ColorLed.LED_ON)
    robot.head.right_led.setcolor(color=color_right, blink=ColorLed.LED_ON)


def calibrate_controller(joystick, samples=10):
    """Calibrate controller by averaging resting positions"""
    print("Calibrating controller... don't touch it!")
    time.sleep(0.5)
    
    offsets = {
        'lx': 0,      # axis 0
        'rx': 0,      # axis 3
        'l2': 0,      # axis 2
        'r2': 0       # axis 5
    }
    
    for _ in range(samples):
        pygame.event.pump()
        offsets['lx'] += joystick.get_axis(0)
        offsets['rx'] += joystick.get_axis(3)
        offsets['l2'] += joystick.get_axis(2)
        offsets['r2'] += joystick.get_axis(5)
        time.sleep(0.05)
    
    # Average the samples
    for key in offsets:
        offsets[key] /= samples
    
    print(f"Calibration complete!")
    print(f"Offsets - LX: {offsets['lx']:.3f}, RX: {offsets['rx']:.3f}, L2: {offsets['l2']:.3f}, R2: {offsets['r2']:.3f}")
    
    return offsets


class CameraThread(threading.Thread):
    """Thread to handle camera streaming"""
    def __init__(self):
        super().__init__()
        self.running = False
        self.daemon = True
        
    def run(self):
        if not CAMERA_AVAILABLE:
            print("Camera not available, skipping camera thread.")
            return
            
        try:
            # Initialize OpenNI2
            openni2.initialize("/home/thom/saraJvB-main/OpenCV/CameraUbuntu/AstraSDK-v2.1.3/lib/Plugins/openni2/")
            
            # Open the device (ASTRA camera)
            dev = openni2.Device.open_any()
            
            # Create streams
            depth_stream = dev.create_depth_stream()
            color_stream = dev.create_color_stream()
            
            if depth_stream is None:
                print("Depth stream not available.")
                return
            
            print("Camera streams initialized.")
            depth_stream.start()
            color_stream.start()
            
            self.running = True
            
            try:
                while self.running:
                    # Read depth frame
                    depth_frame = depth_stream.read_frame()
                    if depth_frame is not None:
                        depth_data = depth_frame.get_buffer_as_uint16()
                        depth_array = np.ctypeslib.as_array(depth_data)
                        depth_image = depth_array.reshape((depth_frame.height, depth_frame.width))
                        depth_image_8bit = cv2.flip(255 - cv2.convertScaleAbs(depth_image, alpha=0.03), 1)
                        depth_colored = cv2.applyColorMap(depth_image_8bit, cv2.COLORMAP_JET)
                        cv2.imshow("Depth", depth_colored)
                    
                    # Read color frame
                    color_frame = color_stream.read_frame()
                    if color_frame is not None:
                        color_data = color_frame.get_buffer_as_uint8()
                        color_array = np.ctypeslib.as_array(color_data)
                        color_image = cv2.cvtColor(
                            color_array.reshape((color_frame.height, color_frame.width, 3)),
                            cv2.COLOR_RGB2BGR
                        )
                        color_image = cv2.flip(color_image, 1)
                        cv2.imshow("Color", color_image)
                    
                    # Check for 'q' key to stop camera
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.running = False
                        
            finally:
                depth_stream.stop()
                color_stream.stop()
                openni2.unload()
                cv2.destroyAllWindows()
        except Exception as e:
            print(f"Camera error: {e}")
            
    def stop(self):
        self.running = False


def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No controller detected")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller connected: {joystick.get_name()}")

    # Calibrate controller
    offsets = calibrate_controller(joystick)

    robot = SaraRobot(logging=False)
    time.sleep(1.0)

    # Start camera thread if available
    camera_thread = None
    if CAMERA_AVAILABLE:
        camera_thread = CameraThread()
        camera_thread.start()
        print("Camera thread started. Press 'q' in camera window to stop camera.")
    else:
        print("Running without camera.")

    lamp_on = False
    police_mode = False
    police_state = False
    last_police_toggle = 0
    police_interval = 0.5

    forward_speed = 200        # Speed for forward/backward movement
    sideways_speed = 60        # Speed for strafing (separate, lower)
    rotation_speed = -70       # Speed for rotation
    refresh_rate = 0.05

    button_last = {}

    def check_button(btn, name):
        pressed = joystick.get_button(btn)
        was = button_last.get(name, False)
        button_last[name] = pressed
        return pressed and not was

    robot.head.lamp.set_lamp(False)

    try:
        while True:
            pygame.event.pump()

            # Read raw joystick values
            lx_raw = joystick.get_axis(0) - offsets['lx']
            lx = apply_deadzone(lx_raw)
            
            rx_raw = joystick.get_axis(3) - offsets['rx']
            rx = apply_deadzone(rx_raw)

            # L2 trigger = backward (axis 2)
            l2_axis = joystick.get_axis(2)
            l2_normalized = (l2_axis - offsets['l2'] + 1.0) / 2.0
            l2_normalized = max(0.0, min(1.0, l2_normalized))
            l2 = apply_trigger_deadzone(l2_normalized)
            
            # R2 trigger = forward (axis 5)
            r2_axis = joystick.get_axis(5)
            r2_normalized = (r2_axis - offsets['r2'] + 1.0) / 2.0
            r2_normalized = max(0.0, min(1.0, r2_normalized))
            r2 = apply_trigger_deadzone(r2_normalized)

            # Calculate forward movement
            forward = int((r2 - l2) * forward_speed)
            
            # Check if we're moving forward/backward
            is_moving_forward_backward = (forward != 0)
            
            if is_moving_forward_backward:
                # When moving forward/backward:
                # - Left stick = rotation (REVERSED for car-like steering)
                # - Right stick = strafe (REVERSED)
                rotation = int(-lx * rotation_speed)  # Negative to reverse
                sideways = int(-rx * sideways_speed)  # Negative to reverse
            else:
                # When stationary:
                # - Left stick = strafe
                # - Right stick = rotation
                sideways = int(lx * sideways_speed)
                rotation = int(rx * rotation_speed)
            
            # Clamp rotation to safe range
            rotation = max(-100, min(100, rotation))

            # R1 → flashlight (button 5)
            if check_button(5, "lamp"):
                lamp_on = not lamp_on
                robot.head.lamp.set_lamp(lamp_on)
                print(f"Lamp {'ON' if lamp_on else 'OFF'}")

            # L1 → police lights (button 4)
            if check_button(4, "police"):
                police_mode = not police_mode
                print(f"Police mode {'ON' if police_mode else 'OFF'}")
                if not police_mode:
                    for led in [
                        robot.left_arm.led,
                        robot.right_arm.led,
                        robot.base.led,
                        robot.head.left_led,
                        robot.head.right_led,
                    ]:
                        led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)

            # Police flashing
            if police_mode:
                now = time.time()
                if now - last_police_toggle >= police_interval:
                    last_police_toggle = now
                    police_state = not police_state

                if police_state:
                    set_police_leds(robot, ColorLed.RED, ColorLed.BLUE)
                else:
                    set_police_leds(robot, ColorLed.BLUE, ColorLed.RED)

            # Movement - only send command if there's actual input
            if forward == 0 and sideways == 0 and rotation == 0:
                robot.base.move_stop()
            else:
                robot.base.move(
                    Sideways_Velocity=sideways,
                    Forward_Velocity=forward,
                    Rotation_Velocity=rotation,
                )

            # Share → quit
            if joystick.get_button(8):
                print("Exiting...")
                break

            time.sleep(refresh_rate)

    finally:
        # Stop camera thread if it exists
        if camera_thread is not None:
            camera_thread.stop()
            camera_thread.join(timeout=2.0)
        
        # Stop robot
        robot.base.move_stop()
        robot.head.lamp.set_lamp(False)

        for led in [
            robot.left_arm.led,
            robot.right_arm.led,
            robot.base.led,
            robot.head.left_led,
            robot.head.right_led,
        ]:
            led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)

        robot.base.brake(ApplyBrake=False)
        pygame.quit()
        robot.stop()


if __name__ == "__main__":
    main()