"""
Red ball tracking with Sara head
Compatible with Sara firmware: step-based blocking moves
"""

import time
import numpy as np
import cv2
from openni import openni2

from Common.sara_library import SaraRobot
from Common.sara_common import RobotHeadPositions

# =========================
# CAMERA INIT
# =========================
openni2.initialize(
    "/home/thom/saraJvB-main/OpenCV/CameraUbuntu/AstraSDK-v2.1.3/lib/Plugins/openni2/"
)
dev = openni2.Device.open_any()
color_stream = dev.create_color_stream()
color_stream.start()

# =========================
# COLOR DETECTION PARAMETERS
# =========================
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 100, 100])
upper_red2 = np.array([180, 255, 255])
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# =========================
# MOTOR SETTINGS
# =========================
PAN_CENTER = RobotHeadPositions.PAN_MID
TILT_CENTER = RobotHeadPositions.TILT_MID
PAN_MIN, PAN_MAX = 0, 500
TILT_MIN, TILT_MAX = 0, 500

PIXELS_TO_MOTOR = 0.5  # scale factor: pixels -> motor units
STEP_SLEEP = 0.8       # seconds to allow motor to move

# =========================
# MAIN FUNCTION
# =========================
def main():
    robot = SaraRobot(logging=False)
    time.sleep(1)

    # -------------------
    # HANDSHAKE REQUIRED
    # -------------------
    robot.head.getversion()
    robot.body.getversion()
    time.sleep(0.5)

    # Start centered
    pan = PAN_CENTER
    tilt = TILT_CENTER
    robot.head.pan_motor.move(position=pan)
    robot.head.tilt_motor.move(position=tilt)
    time.sleep(1)

    try:
        while True:
            # ---------- READ FRAME ----------
            frame_data = color_stream.read_frame()
            color_data = frame_data.get_buffer_as_uint8()
            frame = np.ctypeslib.as_array(color_data)
            frame = frame.reshape((frame_data.height, frame_data.width, 3))
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame = cv2.flip(frame, 1)

            h, w, _ = frame.shape
            cx_img = w // 2
            cy_img = h // 2

            # ---------- DETECT RED BALL ----------
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                cnt = max(contours, key=cv2.contourArea)
                if cv2.contourArea(cnt) > 150:
                    M = cv2.moments(cnt)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # ---------- MAP PIXELS TO MOTOR ----------
                        new_pan = PAN_CENTER - int((cx - cx_img) * PIXELS_TO_MOTOR)
                        new_tilt = TILT_CENTER - int((cy - cy_img) * PIXELS_TO_MOTOR)

                        new_pan = max(PAN_MIN, min(PAN_MAX, new_pan))
                        new_tilt = max(TILT_MIN, min(TILT_MAX, new_tilt))

                        # Only move if different
                        if new_pan != pan or new_tilt != tilt:
                            robot.head.pan_motor.move(position=new_pan)
                            robot.head.tilt_motor.move(position=new_tilt)
                            pan, tilt = new_pan, new_tilt
                            time.sleep(STEP_SLEEP)  # IMPORTANT: allow motor to move

                        # Draw ball
                        cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)

            # Draw center
            cv2.circle(frame, (cx_img, cy_img), 6, (255, 0, 0), -1)

            # ---------- DISPLAY ----------
            display = np.hstack([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
            cv2.putText(display, f"PAN:{pan} TILT:{tilt}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Sara Red Ball Tracking", display)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        robot.stop()
        color_stream.stop()
        openni2.unload()
        cv2.destroyAllWindows()

# =========================
# RUN
# =========================
if __name__ == "__main__":
    main()
