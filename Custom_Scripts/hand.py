import time
import cv2
import numpy as np
from openni import openni2
import mediapipe as mp
from Common.sara_library import SaraRobot
from Common.sara_common import RobotHeadPositions

# -------------------------------------------------------
# Initialize robot
# -------------------------------------------------------
print("--------------------------------------------------------------------------------")
print("Starting robot")
print("--------------------------------------------------------------------------------")
robot = SaraRobot(logging=False)
time.sleep(1)
robot.head.getversion()
robot.body.getversion()

# Safe range limits (adjust if needed)
PAN_MIN, PAN_MAX = 100, 400
TILT_MIN, TILT_MAX = 150, 450

# Speed tuning
PAN_SPEED = 1.0
TILT_SPEED = 1.0
DAMPING = 0.3  # smooth motion (0.1 = slow, 1 = instant)
INVERT_PAN = True   # Set True if it looks away horizontally
INVERT_TILT = False # Set True if it looks away vertically

# Initialize camera (Astra)
openni2.initialize("/home/thom/saraJvB-main/OpenCV/CameraUbuntu/AstraSDK-v2.1.3/lib/Plugins/openni2/")
dev = openni2.Device.open_any()
color_stream = dev.create_color_stream()
color_stream.start()

# Mediapipe setup
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.5)

print("✅ Astra camera + Mediapipe hand tracking initialized")

# Initial servo positions
pan_pos = RobotHeadPositions.PAN_MID
tilt_pos = RobotHeadPositions.TILT_MID

robot.head.pan_motor.move(pan_pos)
robot.head.tilt_motor.move(tilt_pos)

# -------------------------------------------------------
# Main loop
# -------------------------------------------------------
try:
    while True:
        # Capture RGB frame
        color_frame = color_stream.read_frame()
        color_data = color_frame.get_buffer_as_uint8()
        color_array = np.ctypeslib.as_array(color_data)
        frame = color_array.reshape((color_frame.height, color_frame.width, 3))
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.flip(frame, 1)

        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2
        cv2.drawMarker(frame, (center_x, center_y), (0, 255, 255),
                       markerType=cv2.MARKER_CROSS, thickness=1)

        # Process Mediapipe
        result = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if result.multi_hand_landmarks:
            hand_landmarks = result.multi_hand_landmarks[0]
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            wrist = hand_landmarks.landmark[0]
            x = int(wrist.x * w)
            y = int(wrist.y * h)
            cv2.circle(frame, (x, y), 10, (0, 0, 255), 2)

            dx = (x - center_x) / center_x
            dy = (y - center_y) / center_y
            
            # Apply direction inversions
            if INVERT_PAN:
                dx = -dx
            if INVERT_TILT:
                dy = -dy
            
            target_pan = pan_pos + int(dx * 50 * PAN_SPEED)
            target_tilt = tilt_pos + int(dy * 25 * TILT_SPEED)


            # Clamp within safe range
            target_pan = max(PAN_MIN, min(PAN_MAX, target_pan))
            target_tilt = max(TILT_MIN, min(TILT_MAX, target_tilt))

            # Smooth damping
            pan_pos = int(pan_pos + (target_pan - pan_pos) * DAMPING)
            tilt_pos = int(tilt_pos + (target_tilt - tilt_pos) * DAMPING)

            try:
                robot.head.pan_motor.move(pan_pos)
                robot.head.tilt_motor.move(tilt_pos)
            except Exception as e:
                print("⚠️ Motor move skipped:", e)

            cv2.putText(frame, f"PAN {pan_pos} | TILT {tilt_pos}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255), 2)

        else:
            # Gently return to center if no hand detected
            pan_pos = int(pan_pos + (RobotHeadPositions.PAN_MID - pan_pos) * 0.05)
            tilt_pos = int(tilt_pos + (RobotHeadPositions.TILT_MID - tilt_pos) * 0.05)
            try:
                robot.head.pan_motor.move(pan_pos)
                robot.head.tilt_motor.move(tilt_pos)
            except Exception:
                pass

        # Show live feedback
        cv2.imshow("Astra Hand Tracking", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    pass
finally:
    print("Disconnecting")
    color_stream.stop()
    openni2.unload()
    robot.stop()
    cv2.destroyAllWindows()
