import cv2
import numpy as np
from openni import openni2
import random
import time

# Initialize OpenNI2
openni2.initialize("E:/saraJvB-main/saraJvB-main/OpenCV/CameraWindows/AstraSDK-v2.1.3-Win64/bin")

# Open the device
dev = openni2.Device.open_any()

# Start depth stream
depth_stream = dev.create_depth_stream()
depth_stream.start()

# Pong parameters
width, height = 640, 480
paddle_h, paddle_w = 80, 15
ball_r = 10
ball_speed_x, ball_speed_y = 6, 4
player_y = height // 2 - paddle_h // 2
ai_y = height // 2 - paddle_h // 2
ball_x, ball_y = width // 2, height // 2
score_player, score_ai = 0, 0

# Depth parameters
depth_min, depth_max = 600, 1800  # tune this for your hand range
alpha = 0.05  # contrast multiplier

# Function to get hand Y position from depth
def get_hand_position(depth_image):
    # Focus on a central region to reduce noise
    roi = depth_image[100:400, 100:540]
    mask = (roi > depth_min) & (roi < depth_max)
    if np.count_nonzero(mask) < 500:
        return None  # no hand detected
    y_coords, x_coords = np.where(mask)
    avg_y = np.mean(y_coords)
    return int(avg_y / roi.shape[0] * height)

# Main loop
try:
    while True:
        # Read depth frame
        depth_frame = depth_stream.read_frame()
        depth_data = depth_frame.get_buffer_as_uint16()
        depth_array = np.ctypeslib.as_array(depth_data)
        depth_image = depth_array.reshape((depth_frame.height, depth_frame.width))

        # Flip + normalize for display
        depth_image = cv2.flip(depth_image, 1)
        depth_8 = cv2.convertScaleAbs(depth_image, alpha=alpha)
        depth_colored = cv2.applyColorMap(255 - depth_8, cv2.COLORMAP_JET)

        # Get hand position
        hand_y = get_hand_position(depth_image)
        if hand_y is not None:
            player_y = int(np.clip(hand_y - paddle_h // 2, 0, height - paddle_h))

        # Move AI paddle
        if ai_y + paddle_h // 2 < ball_y:
            ai_y += 4
        elif ai_y + paddle_h // 2 > ball_y:
            ai_y -= 4

        ai_y = int(np.clip(ai_y, 0, height - paddle_h))

        # Move ball
        ball_x += ball_speed_x
        ball_y += ball_speed_y

        # Bounce on top/bottom
        if ball_y <= 0 or ball_y >= height:
            ball_speed_y *= -1

        # Bounce on paddles
        if (ball_x - ball_r <= paddle_w and player_y < ball_y < player_y + paddle_h):
            ball_speed_x *= -1
            ball_x = paddle_w + ball_r
        elif (ball_x + ball_r >= width - paddle_w and ai_y < ball_y < ai_y + paddle_h):
            ball_speed_x *= -1
            ball_x = width - paddle_w - ball_r

        # Score
        if ball_x < 0:
            score_ai += 1
            ball_x, ball_y = width // 2, height // 2
            time.sleep(0.5)
        elif ball_x > width:
            score_player += 1
            ball_x, ball_y = width // 2, height // 2
            time.sleep(0.5)

        # Draw game
        game = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.rectangle(game, (0, player_y), (paddle_w, player_y + paddle_h), (0, 255, 0), -1)
        cv2.rectangle(game, (width - paddle_w, ai_y), (width, ai_y + paddle_h), (0, 0, 255), -1)
        cv2.circle(game, (ball_x, ball_y), ball_r, (255, 255, 255), -1)
        cv2.putText(game, f"{score_player} : {score_ai}", (width // 2 - 40, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Overlay the game on the depth view (optional, looks awesome)
        overlay = cv2.addWeighted(depth_colored, 0.5, game, 1.0, 0)

        cv2.imshow("Depth Pong 🏓", overlay)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    depth_stream.stop()
    openni2.unload()
    cv2.destroyAllWindows()
