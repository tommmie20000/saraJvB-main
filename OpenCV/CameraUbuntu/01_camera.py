#------------------------------------------------------------------
# Unpack the tar.gz archive
#------------------------------------------------------------------
# tar -xvf AstraSDK-v2.1.3-94.tar.gz

#------------------------------------------------------------------
# Bouwen van de samples moet tegen de huidige video driver ofzo.
# Niet essentieel voor de python scripts.
#------------------------------------------------------------------
# cd samples/
# mkdir build && cd build
# cmake ..
# make -j$(nproc)

#------------------------------------------------------------------
# Install the USB rules to set user / group correctly for video
#------------------------------------------------------------------
# Install the usb rules
# cd install/
# chmod +x install.sh
# ./install.sh

#------------------------------------------------------------------
# Add current user to video group
#------------------------------------------------------------------
# sudo adduser $USER video

#------------------------------------------------------------------
# Zet path naar de openni2 library
#------------------------------------------------------------------
# openni2.initialize("/home/matthijs/Downloads/AstraSDK-v2.1.3-94/lib/Plugins/openni2/") # Path to OpenNI2 libs

from openni import openni2
import numpy as np
import cv2

# Initialize OpenNI2
openni2.initialize("/home/laptopschool/code/sarajvb/'saraJvB-main (2)'/3D_Camera/AstraSDK-v2.1.3-Ubuntu-x86_64/AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64/lib/Plugins/openni2/") # Path to OpenNI2 libs

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
