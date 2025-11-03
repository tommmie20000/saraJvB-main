#-----------------------------------------------------------------------------------
# Installation instructions
# https://www.orbbec.com/developers/astra-sdk
#-----------------------------------------------------------------------------------
# Install the USB driver for the camera 
# Astra-win32-driver-4.3.0.22.zip
# 
# # Unzip the AstraSDK-v2.1.3-Win64.zip
# Move all files 1 level up in the new folder or use the really long name.
# 
# /AstraSDK-v2.1.3-Win64/AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64/bin
# OR
# /AstraSDK-v2.1.3-Win64/bin
# 

import numpy as np
from openni import openni2
import cv2
import sys

# Print Python executable and path for debugging
print("Python executable:", sys.executable)
print("Python path:", sys.path)

# Initialize OpenNI2
openni2.initialize("E:/saraJvB-main/saraJvB-main/OpenCV/CameraWindows/AstraSDK-v2.1.3-Win64/bin")  # Update with actual path to OpenNI2.dll

# Open the device (ASTRA camera)
dev = openni2.Device.open_any()

# Create a depth stream
depth_stream = dev.create_depth_stream()

# Check if the depth stream is available
if depth_stream is None:
    print("Depth stream not available.")
else:
    print("Depth stream found.")

depth_stream.start()

# Create a color stream
color_stream = dev.create_color_stream()
color_stream.start()

try:
    while True:
        # Read a frame from the depth stream
        depth_frame = depth_stream.read_frame()
        depth_data = depth_frame.get_buffer_as_uint16()

        if depth_frame is None:
            print("No depth frame received.")
            continue

        # Convert the depth data (ctypes array) to a NumPy array
        depth_array = np.ctypeslib.as_array(depth_data)

        if depth_data is None or len(depth_data) == 0:
            print("Empty depth data.")
            continue

        # Reshape the array to match the frame's dimensions (height, width)
        depth_image = depth_array.reshape(
            (depth_frame.height, depth_frame.width)
        )

        depth_image_8bit = cv2.flip(
            255 - cv2.convertScaleAbs(depth_image, alpha=0.03), 1
        )

        # Map depth to color: far = blue, close = red
        depth_colored = cv2.applyColorMap(depth_image_8bit, cv2.COLORMAP_JET)

        # Read a frame from the color stream
        color_frame = color_stream.read_frame()
        color_data = color_frame.get_buffer_as_uint8()

        # Convert the color data (ctypes array) to a NumPy array
        color_array = np.ctypeslib.as_array(color_data)

        # Reshape the array to match the frame's dimensions (height, width, 3 channels)
        color_image = cv2.cvtColor(
            color_array.reshape((color_frame.height, color_frame.width, 3)),
            cv2.COLOR_RGB2BGR,
        )

        color_image = cv2.flip(color_image, 1)

        # color_image = color_array.reshape(
        #     (color_frame.height, color_frame.width, 3)
        # )

        # Show the depth and color images
        cv2.imshow("Depth", depth_colored)
        cv2.imshow("Color", color_image)

        # Exit if the user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    # Clean up
    depth_stream.stop()
    color_stream.stop()
    openni2.unload()
    cv2.destroyAllWindows()
