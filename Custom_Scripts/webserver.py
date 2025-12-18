from flask import Flask, Response, render_template_string, request
import cv2
import argparse
import time
import numpy as np
from openni import openni2

#!/usr/bin/env python3
"""
webserver.py

Simple Flask webserver that serves an index page with a live MJPEG video feed
from a local camera (OpenCV). Run this file and open http://<host>:8000/

Requirements:
    pip install flask opencv-python

Usage:
    python webserver.py --host 0.0.0.0 --port 8000 --camera 0
"""

app = Flask(__name__)

INDEX_HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Live Feed</title>
  <style>
    body { margin: 0; background: #111; color: #eee; display:flex; height:100vh;
           align-items:center; justify-content:center; flex-direction:column; }
    img { max-width: 100%; height: auto; border: 4px solid #222; box-shadow: 0 0 20px rgba(0,0,0,0.5); }
    .info { margin-top: 10px; font-family: Arial, sans-serif; font-size: 14px; color: #bbb; }
  </style>
</head>
<body>
  <img id="feed" src="{{ url_for('video_feed') }}" alt="Live feed">
  <div class="info">If the feed is blank, ensure the camera index is correct and OpenCV has access.</div>
</body>
</html>
"""

def create_camera(source=0):
    # Initialize OpenNI2 for depth camera
    openni2.initialize("E:/saraJvB-main/saraJvB-main/OpenCV/CameraWindows/AstraSDK-v2.1.3-Win64/bin")  # Update with actual path
    dev = openni2.Device.open_any()
    
    # Create color and depth streams
    color_stream = dev.create_color_stream()
    depth_stream = dev.create_depth_stream()
    
    color_stream.start()
    depth_stream.start()
    
    return color_stream, depth_stream

def mjpeg_generator(color_stream, depth_stream):
    try:
        while True:
            # Read color frame
            color_frame = color_stream.read_frame()
            color_data = color_frame.get_buffer_as_uint8()
            color_image = cv2.cvtColor(
                np.ctypeslib.as_array(color_data).reshape((color_frame.height, color_frame.width, 3)),
                cv2.COLOR_RGB2BGR,
            )
            color_image = cv2.flip(color_image, 1)

            # Read depth frame
            depth_frame = depth_stream.read_frame()
            depth_data = depth_frame.get_buffer_as_uint16()
            depth_array = np.ctypeslib.as_array(depth_data).reshape((depth_frame.height, depth_frame.width))
            depth_image_8bit = cv2.flip(255 - cv2.convertScaleAbs(depth_array, alpha=0.03), 1)
            depth_colored = cv2.applyColorMap(depth_image_8bit, cv2.COLORMAP_JET)

            # Encode color image as JPEG
            ok, jpeg_color = cv2.imencode('.jpg', color_image)
            if not ok:
                continue

            # Encode depth image as JPEG
            ok, jpeg_depth = cv2.imencode('.jpg', depth_colored)
            if not ok:
                continue

            # Yield both images as separate streams
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg_color.tobytes() + b'\r\n'
                   b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg_depth.tobytes() + b'\r\n')
    finally:
        try:
            depth_stream.stop()
            color_stream.stop()
            openni2.unload()
        except Exception:
            pass

@app.route('/')
def index():
    return render_template_string(INDEX_HTML)

@app.route('/video_feed')
def video_feed():
    color_stream, depth_stream = create_camera()
    return Response(mjpeg_generator(color_stream, depth_stream),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def main():
    parser = argparse.ArgumentParser(description="Simple MJPEG webserver (Flask + OpenCV)")
    parser.add_argument('--host', default='0.0.0.0', help='Bind host (default 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8000, help='Port (default 8000)')
    parser.add_argument('--camera', default=0, help='Camera index or stream URL (default 0)')
    args = parser.parse_args()

    # store camera source in app config
    try:
        cam_src = int(args.camera)
    except Exception:
        cam_src = args.camera  # e.g. rtsp:// or file path

    app.config['CAM_SOURCE'] = cam_src
    # Use threaded server so multiple clients can connect
    app.run(host=args.host, port=args.port, threaded=True)

if __name__ == '__main__':
    main()