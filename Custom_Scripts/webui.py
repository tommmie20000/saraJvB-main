#!/usr/bin/env python3
"""
robot_webui.py
Single-file Flask web UI:
- color + depth MJPEG streams from OpenNI (Astra)
- websocket controls via Flask-SocketIO
- movement, simple LEDs, lamp, police mode, speed slider
- status panel: movement, LED states, battery (using your Battery class), CPU temp, camera FPS

Run:
pip3 install flask flask-socketio eventlet opencv-python numpy
python3 robot_webui.py
"""

import time
import threading
import io
import os
from datetime import datetime

from flask import Flask, Response, render_template_string, request
from flask_socketio import SocketIO, emit
import eventlet
eventlet.monkey_patch()

import cv2
import numpy as np

# ---------------------------
# Import your robot library pieces
# ---------------------------
try:
    from Common.sara_library import SaraRobot, ColorLed
    # Battery class you provided should already be wired into the SaraRobot instance:
    # robot.body.battery.get_voltage(), robot.body.battery.get_batterystate() etc.
except Exception as e:
    print("Warning: Could not import SaraRobot. Robot calls will fail until import works.")
    SaraRobot = None
    ColorLed = None

# ---------------------------
# OpenNI initialization (adjust path to your SDK)
# ---------------------------
from openni import openni2

OPENNI_PLUGIN_PATH = "/home/sara/Downloads/OpenCV/CameraRaspberry/AstraSDK-v2.1.3-94bca0f52e-20210611T023312Z-Linux-aarch64/lib/Plugins/openni2/"
# If your Pi is 32-bit, change the path accordingly.

openni2.initialize(OPENNI_PLUGIN_PATH)
dev = openni2.Device.open_any()
depth_stream = dev.create_depth_stream()
color_stream = dev.create_color_stream()
depth_stream.start()
color_stream.start()

# ---------------------------
# Flask + SocketIO
# ---------------------------
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode='eventlet')

# ---------------------------
# Global shared state
# ---------------------------
state = {
    "cmd": {"f": 0, "s": 0, "r": 0},   # forward, sideways, rotation (raw values -100..100)
    "speed_scale": 60,                # default speed
    "lamp": False,
    "police_mode": False,
    "led_color": "white",             # "white"/"red"/"blue"
    "last_manual_ts": 0.0,
    "movement_report": {"f": 0, "s": 0, "r": 0},
    "camera_fps": {"color": 0.0, "depth": 0.0},
    "battery": {"voltage": None, "state": None, "current": None},
    "cpu_temp": None,
    "led_states": {"left": "white", "right": "white", "base": "white", "head": "white"},
}

# Robot instance - create once
robot = None
if SaraRobot is not None:
    try:
        robot = SaraRobot(logging=False)
        # Make sure robot has battery etc. If not present, code below handles exceptions.
    except Exception as e:
        print("Warning: failed to instantiate SaraRobot:", e)
        robot = None

# ---------------------------
# Helpers
# ---------------------------
def get_cpu_temp_c():
    # try reading from sysfs
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            t = int(f.read().strip()) / 1000.0
            return round(t, 1)
    except:
        return None

def update_battery_state():
    if robot is None:
        return
    try:
        b = robot.body.battery
        state["battery"]["voltage"] = int(b.get_voltage())
        state["battery"]["state"] = int(b.get_batterystate())
        state["battery"]["current"] = int(b.get_current())
    except Exception:
        # ignore if not available
        pass

def apply_led_choice(color_name):
    """Set simple LEDs as chosen. color_name in {white, red, blue}"""
    if robot is None or ColorLed is None:
        return
    try:
        mapping = {
            "white": ColorLed.WHITE if hasattr(ColorLed, "WHITE") else 0,
            "red": ColorLed.RED if hasattr(ColorLed, "RED") else 1,
            "blue": ColorLed.BLUE if hasattr(ColorLed, "BLUE") else 2,
        }
        col = mapping.get(color_name, mapping.get("white", 0))
        # apply to a few locations (left/right/base/head)
        try:
            robot.left_arm.led.setcolor(col, ColorLed.LED_ON)
            robot.right_arm.led.setcolor(col, ColorLed.LED_ON)
            robot.base.led.setcolor(col, ColorLed.LED_ON)
            robot.head.left_led.setcolor(col, ColorLed.LED_ON)
            robot.head.right_led.setcolor(col, ColorLed.LED_ON)
        except Exception:
            # some robots may use different attribute names - ignore
            pass
        # save state
        state["led_states"]["left"] = color_name
        state["led_states"]["right"] = color_name
        state["led_states"]["base"] = color_name
        state["led_states"]["head"] = color_name
    except Exception:
        pass

def set_lamp(on: bool):
    if robot is None:
        return
    try:
        robot.head.lamp.set_lamp(lamp_on=on)
    except Exception:
        pass

def toggle_police_mode(enable: bool):
    # simple alternating police LEDs
    state["police_mode"] = enable
    if not enable:
        apply_led_choice("white")
    # if enabled, the police thread will handle alternating colors

# ---------------------------
# Police thread
# ---------------------------
def police_thread_fn():
    toggle = False
    while True:
        if state["police_mode"]:
            toggle = not toggle
            if toggle:
                apply_led_choice("red")
            else:
                apply_led_choice("blue")
        time.sleep(0.5)

threading.Thread(target=police_thread_fn, daemon=True).start()

# ---------------------------
# Robot command loop (applies commands to physical robot)
# Runs in background to keep commands applied continuously (safety timeout model)
# ---------------------------
def robot_command_loop():
    """
    Periodically reads state['cmd'] and sends commands to robot.
    """
    global robot
    interval = 0.15  # seconds
    while True:
        cmd = state["cmd"]
        f = int(cmd.get("f", 0))
        s = int(cmd.get("s", 0))
        r = int(cmd.get("r", 0))

        # Save reported movement for UI
        state["movement_report"]["f"] = f
        state["movement_report"]["s"] = s
        state["movement_report"]["r"] = r

        if robot is not None:
            try:
                if f == 0 and s == 0 and r == 0:
                    robot.base.move_stop()
                else:
                    robot.base.move(Sideways_Velocity=s, Forward_Velocity=f, Rotation_Velocity=r)
            except Exception:
                pass
        # emit status to clients occasionally
        try:
            update_battery_state()
            state["cpu_temp"] = get_cpu_temp_c()
            socketio.emit("status_update", {
                "movement": state["movement_report"],
                "led_states": state["led_states"],
                "battery": state["battery"],
                "cpu_temp": state["cpu_temp"],
                "camera_fps": state["camera_fps"],
                "police_mode": state["police_mode"],
            })
        except Exception:
            pass

        time.sleep(interval)

threading.Thread(target=robot_command_loop, daemon=True).start()

# ---------------------------
# Camera generators (MJPEG)
# ---------------------------
def gen_color_mjpeg():
    last_time = time.time()
    frame_count = 0
    while True:
        try:
            cframe = color_stream.read_frame()
            cdata = cframe.get_buffer_as_uint8()
            carr = np.ctypeslib.as_array(cdata)
            img = carr.reshape((cframe.height, cframe.width, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            img = cv2.flip(img, 1)

            # draw small overlay with FPS and timestamp
            frame_count += 1
            now = time.time()
            if now - last_time >= 1.0:
                state["camera_fps"]["color"] = frame_count / (now - last_time)
                frame_count = 0
                last_time = now

            cv2.putText(img, f"FPS: {state['camera_fps']['color']:.1f}",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            _, jpeg = cv2.imencode('.jpg', img)
            data = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + data + b'\r\n')
        except Exception:
            # camera read error, sleep briefly
            time.sleep(0.05)

def gen_depth_mjpeg():
    last_time = time.time()
    frame_count = 0
    while True:
        try:
            dframe = depth_stream.read_frame()
            ddata = dframe.get_buffer_as_uint16()
            darr = np.ctypeslib.as_array(ddata)
            depth = darr.reshape((dframe.height, dframe.width))
            depth_8 = cv2.convertScaleAbs(depth, alpha=0.03)
            depth_8 = 255 - depth_8
            depth_8 = cv2.flip(depth_8, 1)
            depth_col = cv2.applyColorMap(depth_8, cv2.COLORMAP_JET)

            frame_count += 1
            now = time.time()
            if now - last_time >= 1.0:
                state["camera_fps"]["depth"] = frame_count / (now - last_time)
                frame_count = 0
                last_time = now

            cv2.putText(depth_col, f"FPS: {state['camera_fps']['depth']:.1f}",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

            _, jpeg = cv2.imencode('.jpg', depth_col)
            data = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + data + b'\r\n')
        except Exception:
            time.sleep(0.05)

# ---------------------------
# Flask routes
# ---------------------------
INDEX_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Robot Control - Control Panel</title>
  <style>
    body { background:#0b0b0b; color:white; font-family: Arial, Helvetica, sans-serif; margin:0; padding:10px;}
    .top { display:flex; gap:10px; }
    .cam { flex:1; background:#222; padding:8px; border-radius:6px; }
    img.stream { width:100%; height:auto; border-radius:4px; display:block; }
    .rightpane { width:360px; background:#121212; padding:12px; border-radius:6px; }
    .section { margin-bottom:12px; padding:8px; background:#0f0f0f; border-radius:6px;}
    button { padding:8px 12px; margin:4px; border-radius:6px; background:#1e1e1e; color:white; border:1px solid #333; cursor:pointer;}
    button:active { transform: translateY(1px); }
    .big { font-size:18px; padding:10px 16px; }
    .status-row { display:flex; justify-content:space-between; padding:6px 0; }
    .control-grid { display:flex; flex-wrap:wrap; gap:6px; }
    .slider { width:100%; }
    .kbd-hint { color:#bbb; font-size:12px; }
  </style>
</head>
<body>
  <h2>Robot Control — Control Panel</h2>
  <div class="top">
    <div class="cam">
      <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px;">
        <strong>Color Camera</strong>
        <span id="color_fps">FPS: --</span>
      </div>
      <img id="color_stream" class="stream" src="/color_feed">
    </div>

    <div class="cam">
      <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px;">
        <strong>Depth Camera</strong>
        <span id="depth_fps">FPS: --</span>
      </div>
      <img id="depth_stream" class="stream" src="/depth_feed">
    </div>

    <div class="rightpane">
      <div class="section">
        <strong>Movement</strong>
        <div style="margin-top:8px;">
          <div class="control-grid">
            <button id="btn_forward" class="big">Forward (W)</button>
            <button id="btn_back" class="big">Back (S)</button>
            <button id="btn_left" class="big">Rotate Left (A)</button>
            <button id="btn_right" class="big">Rotate Right (D)</button>
            <button id="btn_strafe_l" class="big">Strafe L (Q)</button>
            <button id="btn_strafe_r" class="big">Strafe R (E)</button>
            <button id="btn_stop" class="big" style="background:#a22;">STOP</button>
          </div>
        </div>
        <div style="margin-top:8px;">
          <label>Speed: <span id="speed_val">60</span></label>
          <input id="speed_slider" class="slider" type="range" min="10" max="100" value="60">
        </div>
        <div style="margin-top:8px;">
          <div class="kbd-hint">Keyboard controls: W/S/A/D + Q/E. Hold to move; release stops.</div>
        </div>
      </div>

      <div class="section">
        <strong>LEDs & Lamp</strong>
        <div style="margin-top:8px;">
          <div>
            <button id="led_white">White</button>
            <button id="led_red">Red</button>
            <button id="led_blue">Blue</button>
            <button id="police_btn">Police</button>
          </div>
          <div style="margin-top:8px;">
            <button id="lamp_btn">Toggle Lamp</button>
          </div>
        </div>
      </div>

      <div class="section">
        <strong>Status</strong>
        <div class="status-row"><span>Movement</span><span id="status_movement">f:0 s:0 r:0</span></div>
        <div class="status-row"><span>LEDs</span><span id="status_leds">white</span></div>
        <div class="status-row"><span>Battery</span><span id="status_batt">-- mV</span></div>
        <div class="status-row"><span>CPU Temp</span><span id="status_cpu">-- °C</span></div>
        <div class="status-row"><span>Camera FPS</span><span id="status_camfps">C:-- / D:--</span></div>
      </div>

    </div>
  </div>

<script src="//cdnjs.cloudflare.com/ajax/libs/socket.io/4.5.4/socket.io.min.js"></script>
<script>
  const socket = io();

  // local command state
  let cmd = {f:0, s:0, r:0};
  let speed = 60;
  let keys = {};

  function send_cmd() {
    const scaled = {
      f: Math.round(cmd.f),
      s: Math.round(cmd.s),
      r: Math.round(cmd.r)
    };
    socket.emit("manual_cmd", scaled);
    document.getElementById("status_movement").innerText = `f:${scaled.f} s:${scaled.s} r:${scaled.r}`;
  }

  // key handlers
  document.addEventListener("keydown", (e) => {
    if (e.repeat) return;
    keys[e.key.toLowerCase()] = true;
    update_from_keys();
  });

  document.addEventListener("keyup", (e) => {
    keys[e.key.toLowerCase()] = false;
    update_from_keys();
  });

  function update_from_keys() {
    // reset
    cmd.f = 0; cmd.s = 0; cmd.r = 0;
    if (keys['w']) cmd.f = speed;
    if (keys['s']) cmd.f = -speed;
    if (keys['a']) cmd.r = 60;
    if (keys['d']) cmd.r = -60;
    if (keys['q']) cmd.s = -speed;
    if (keys['e']) cmd.s = speed;
    send_cmd();
  }

  // buttons
  document.getElementById("btn_forward").addEventListener("mousedown", () => { keys['w']=true; update_from_keys(); });
  document.getElementById("btn_forward").addEventListener("mouseup", () => { keys['w']=false; update_from_keys(); });
  document.getElementById("btn_back").addEventListener("mousedown", () => { keys['s']=true; update_from_keys(); });
  document.getElementById("btn_back").addEventListener("mouseup", () => { keys['s']=false; update_from_keys(); });
  document.getElementById("btn_left").addEventListener("mousedown", () => { keys['a']=true; update_from_keys(); });
  document.getElementById("btn_left").addEventListener("mouseup", () => { keys['a']=false; update_from_keys(); });
  document.getElementById("btn_right").addEventListener("mousedown", () => { keys['d']=true; update_from_keys(); });
  document.getElementById("btn_right").addEventListener("mouseup", () => { keys['d']=false; update_from_keys(); });
  document.getElementById("btn_strafe_l").addEventListener("mousedown", () => { keys['q']=true; update_from_keys(); });
  document.getElementById("btn_strafe_l").addEventListener("mouseup", () => { keys['q']=false; update_from_keys(); });
  document.getElementById("btn_strafe_r").addEventListener("mousedown", () => { keys['e']=true; update_from_keys(); });
  document.getElementById("btn_strafe_r").addEventListener("mouseup", () => { keys['e']=false; update_from_keys(); });
  document.getElementById("btn_stop").addEventListener("click", () => { keys={}; cmd={f:0,s:0,r:0}; send_cmd(); });

  // speed slider
  const sld = document.getElementById("speed_slider");
  sld.addEventListener("input", (ev)=> {
    speed = parseInt(ev.target.value);
    document.getElementById("speed_val").innerText = speed;
    update_from_keys();
    socket.emit("speed_change", {speed: speed});
  });

  // led buttons
  document.getElementById("led_white").onclick = () => { socket.emit("led_change", {color:'white'}); }
  document.getElementById("led_red").onclick = () => { socket.emit("led_change", {color:'red'}); }
  document.getElementById("led_blue").onclick = () => { socket.emit("led_change", {color:'blue'}); }
  document.getElementById("police_btn").onclick = () => { socket.emit("police_toggle", {}); }

  document.getElementById("lamp_btn").onclick = () => { socket.emit("lamp_toggle", {}); }

  // Socket events
  socket.on("connect", () => {
    console.log("connected");
  });

  socket.on("status_update", (data) => {
    // update UI status fields
    const leds = data.led_states;
    document.getElementById("status_leds").innerText = `${leds.left}/${leds.right}/${leds.base}/${leds.head}`;
    if (data.battery && data.battery.voltage) {
      document.getElementById("status_batt").innerText = `${data.battery.voltage} mV`;
      document.getElementById("status_batt").innerText = `${data.battery.voltage} mV (${(data.battery.current||'') } mA)`;
    } else {
      document.getElementById("status_batt").innerText = `-- mV`;
    }
    document.getElementById("status_cpu").innerText = data.cpu_temp ? `${data.cpu_temp} °C` : `-- °C`;
    document.getElementById("status_camfps").innerText = `C:${Math.round(data.camera_fps.color||0)}/ D:${Math.round(data.camera_fps.depth||0)}`;
    document.getElementById("color_fps").innerText = `FPS: ${Math.round(data.camera_fps.color||0)}`;
    document.getElementById("depth_fps").innerText = `FPS: ${Math.round(data.camera_fps.depth||0)}`;
  });

</script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(INDEX_HTML)

@app.route('/color_feed')
def color_feed():
    return Response(gen_color_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/depth_feed')
def depth_feed():
    return Response(gen_depth_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ---------------------------
# SocketIO events
# ---------------------------
@socketio.on("manual_cmd")
def handle_manual_cmd(json):
    # Expect {"f": int, "s": int, "r": int}
    try:
        f = int(json.get("f", 0))
        s = int(json.get("s", 0))
        r = int(json.get("r", 0))
        # store to state scaled by speed slider
        speed_factor = state["speed_scale"] / 60.0 if state["speed_scale"] else 1.0
        # keep absolute; UI sends already scaled values; but keep direct assignment
        state["cmd"]["f"] = f
        state["cmd"]["s"] = s
        state["cmd"]["r"] = r
        state["last_manual_ts"] = time.time()
    except Exception as e:
        print("manual_cmd error", e)

@socketio.on("speed_change")
def handle_speed_change(json):
    try:
        sp = int(json.get("speed", 60))
        state["speed_scale"] = sp
    except:
        pass

@socketio.on("led_change")
def handle_led_change(json):
    color = json.get("color","white")
    state["led_color"] = color
    apply_led_choice(color)
    emit("status_update", {
        "led_states": state["led_states"],
        "battery": state["battery"],
        "cpu_temp": state["cpu_temp"],
        "camera_fps": state["camera_fps"],
        "police_mode": state["police_mode"],
    }, broadcast=True)

@socketio.on("lamp_toggle")
def handle_lamp_toggle():
    state["lamp"] = not state["lamp"]
    set_lamp(state["lamp"])
    emit("status_update", {
        "led_states": state["led_states"],
        "battery": state["battery"],
        "cpu_temp": state["cpu_temp"],
        "camera_fps": state["camera_fps"],
        "police_mode": state["police_mode"],
    }, broadcast=True)

@socketio.on("police_toggle")
def handle_police_toggle():
    state["police_mode"] = not state["police_mode"]
    emit("status_update", {
        "led_states": state["led_states"],
        "battery": state["battery"],
        "cpu_temp": state["cpu_temp"],
        "camera_fps": state["camera_fps"],
        "police_mode": state["police_mode"],
    }, broadcast=True)

# ---------------------------
# Run
# ---------------------------
if __name__ == '__main__':
    print("Starting robot_webui on http://0.0.0.0:5000")
    socketio.run(app, host="0.0.0.0", port=5000)
