import time
import math
import numpy as np
import matplotlib.pyplot as plt
import keyboard  # pip install keyboard
from Common.sara_library import SaraRobot, ColorLed

# -------------------------
# TUNABLES (quick access)
# -------------------------
BASE_DIAMETER_CM = 40.0
SENSOR_COUNT = 8
SENSOR_RANGE_CM = 55.0
MIN_OBJECT_WIDTH_CM = 3.0
PASSABLE_MARGIN_CM = 20.0

# Default slips/recov used if calibration fails
DEFAULT_ENCODER_SLIP_DELTA_THRESHOLD = 4.0
DEFAULT_SLIP_MIN_COMMAND_MAG = 6
DEFAULT_SLIP_DETECTION_WINDOW = 0.25

# Recovery parameters (can keep fixed)
RECOVERY_REVERSE_SPEED = -22
RECOVERY_REVERSE_TIME = 0.28
RECOVERY_ROTATE_DEG = 22
RECOVERY_ROTATE_TIMEOUT = 12
# -------------------------


# === Improved Smart avoidance brain (geometry-aware) ===
class SmartAvoidance:
    def __init__(
        self,
        base_diameter_cm=40.0,
        sensor_count=8,
        sensor_range_cm=55.0,
        min_object_width_cm=3.0,
        passable_margin_cm=20.0,
        memory_decay=0.985,
        sensitivity=1,
        side_gain=18.0,
        rot_gain=16.0,
        forward_scale=28,
        memory_weight=0.8,
    ):
        self.base_diameter_cm = float(base_diameter_cm)
        self.sensor_count = int(sensor_count)
        self.sensor_range_cm = float(sensor_range_cm)
        self.min_object_width_cm = float(min_object_width_cm)
        self.passable_margin_cm = float(passable_margin_cm)
        self.min_passable_gap_cm = self.base_diameter_cm + self.passable_margin_cm

        # tuning
        self.memory_decay = memory_decay
        self.sensitivity = sensitivity
        self.side_gain = side_gain
        self.rot_gain = rot_gain
        self.forward_scale = forward_scale
        self.memory_weight = memory_weight

        self.direction_memory = np.zeros(self.sensor_count, dtype=float)
        self.sensor_angles = np.linspace(-90.0, 90.0, self.sensor_count) * math.pi / 180.0

    def _update_sensor_layout(self, sensor_count):
        if sensor_count != self.sensor_count:
            self.sensor_count = int(sensor_count)
            self.direction_memory = np.zeros(self.sensor_count, dtype=float)
            self.sensor_angles = np.linspace(-90.0, 90.0, self.sensor_count) * math.pi / 180.0

    def _values_to_distances(self, sensor_values):
        vals = np.clip(np.asarray(sensor_values, dtype=float), 0.0, 1.0)
        distances = vals * self.sensor_range_cm
        return distances

    def _compute_repulsion_from_distances(self, distances):
        proximity = np.clip((self.sensor_range_cm - distances) / max(1e-6, self.sensor_range_cm), 0.0, 1.0)
        repulsion = (proximity ** 1.9) * self.sensitivity
        repulsion = np.clip(repulsion, 0.0, 1.0)
        return repulsion

    def _inject_virtual_obstacles_for_small_gaps(self, distances, repulsion):
        n = len(distances)
        vr = repulsion.copy()
        angles = self.sensor_angles
        clear_threshold = 0.85 * self.sensor_range_cm

        i = 0
        while i < n:
            if distances[i] >= clear_threshold:
                j = i + 1
                while j < n and distances[j] >= clear_threshold:
                    j += 1
                run_len = j - i
                if run_len >= 1:
                    left_angle = angles[i]
                    right_angle = angles[j - 1]
                    angle_span = abs(right_angle - left_angle)
                    dist_at_gap = min(distances[i], distances[j - 1])
                    if dist_at_gap <= 1e-3:
                        estimated_gap_cm = 0.0
                    else:
                        estimated_gap_cm = 2.0 * dist_at_gap * math.tan(angle_span / 2.0 + 1e-9)
                    if estimated_gap_cm < self.min_passable_gap_cm:
                        for k in range(i, j):
                            center_dist = abs((i + j - 1) / 2.0 - k)
                            norm_center = 1.0 - (center_dist / max(1.0, (run_len - 1) / 2.0))
                            vr[k] = np.clip(vr[k] + 0.6 * norm_center, 0.0, 1.0)
                i = j
            else:
                i += 1

        if n >= 3:
            for k in range(1, n - 1):
                left_close = distances[k - 1] < 0.6 * self.sensor_range_cm
                right_close = distances[k + 1] < 0.6 * self.sensor_range_cm
                middle_clear = distances[k] >= 0.8 * self.sensor_range_cm
                if left_close and right_close and middle_clear:
                    angle_span = abs(angles[k + 1] - angles[k - 1])
                    d = min(distances[k - 1], distances[k + 1])
                    estimated_gap_cm = 2.0 * max(1e-3, d) * math.tan(angle_span / 2.0)
                    if estimated_gap_cm < self.min_passable_gap_cm:
                        vr[k] = np.clip(vr[k] + 0.9, 0.0, 1.0)

        return vr

    def compute(self, sensor_values):
        sensor_values = np.clip(np.asarray(sensor_values, dtype=float), 0.0, 1.0)
        n = len(sensor_values)
        self._update_sensor_layout(n)
        distances = self._values_to_distances(sensor_values)
        repulsion = self._compute_repulsion_from_distances(distances)
        repulsion = self._inject_virtual_obstacles_for_small_gaps(distances, repulsion)

        self.direction_memory = (
            self.direction_memory * self.memory_decay + repulsion * (1.0 - self.memory_decay)
        )

        total_repulsion = repulsion + self.memory_weight * self.direction_memory
        total_repulsion = np.clip(total_repulsion, 0.0, 1.0)

        lateral_force = np.sum(total_repulsion * np.sin(self.sensor_angles))
        frontal_force = np.sum(total_repulsion * np.cos(self.sensor_angles)) / max(1e-3, n)

        sideways = -lateral_force * self.side_gain
        rotation = lateral_force * self.rot_gain
        forward = self.forward_scale * (1.0 - 0.92 * frontal_force)
        forward = float(np.clip(forward, -30.0, self.forward_scale))

        front_indices = np.where(np.abs(self.sensor_angles) <= (30.0 * math.pi / 180.0))[0]
        if len(front_indices) > 0:
            front_min = np.min(sensor_values[front_indices])
            if front_min < 0.18:
                forward = -18.0
                left_min = np.min(sensor_values[: n // 2]) if n // 2 > 0 else 1.0
                right_min = np.min(sensor_values[n // 2 :]) if n // 2 > 0 else 1.0
                if left_min < right_min:
                    rotation = -20.0
                else:
                    rotation = 20.0

        max_side = int(min(80, abs(self.side_gain * 4)))
        max_rot = 80
        max_forward = int(max(0, self.forward_scale))

        sideways_i = int(np.clip(sideways, -max_side, max_side))
        rotation_i = int(np.clip(rotation, -max_rot, max_rot))
        forward_i = int(np.clip(forward, -max_forward, max_forward))

        return forward_i, rotation_i, sideways_i


# === Motion supervisor: uses encoders + compass to detect slip and recover ===
class MotionSupervisor:
    def __init__(
        self,
        robot,
        left_idx=0,
        right_idx=1,
        encoder_slip_delta_threshold=DEFAULT_ENCODER_SLIP_DELTA_THRESHOLD,
        slip_min_command_mag=DEFAULT_SLIP_MIN_COMMAND_MAG,
        slip_detection_window=DEFAULT_SLIP_DETECTION_WINDOW,
    ):
        self.robot = robot
        self.left_idx = left_idx
        self.right_idx = right_idx

        self.enc_last = None
        self.enc_last_time = None
        self.enc_rx_last = None

        self.compass_last = None
        self.compass_last_time = None

        self.slip_since = None
        self.slip_recovering = False

        # calibrated/tunable parameters (might be overwritten by calibrate())
        self.encoder_slip_delta_threshold = encoder_slip_delta_threshold
        self.slip_min_command_mag = slip_min_command_mag
        self.slip_detection_window = slip_detection_window

        # calibration results
        self.expected_enc_delta_per_sec_at_unit_cmd = None  # encoder delta per second per 1 unit forward command
        self.expected_deg_per_sec_at_unit_rot = None  # degrees per second per 1 unit rotation command
        self.compass_noise_deg = None

    def _read_encoders(self):
        try:
            enc = np.array(self.robot.body.encoders.encoders, dtype=float)
            rx = getattr(self.robot.body.encoders, "rx_counter", None)
            return enc, rx
        except Exception:
            return None, None

    def _read_compass(self):
        try:
            c = self.robot.body.compass.read_abs_angle()
            return float(c)
        except Exception:
            return None

    def calibrate(self, verbose=True):
        """
        Run a synchronous calibration:
        1) measure compass noise while stationary
        2) forward encoder response: short forward command -> encoder delta per second per command unit
        3) rotation response: short rotation command -> compass delta per second per rotation-unit
        Use results to set slip threshold and rotation sensitivity.
        """
        print("[Calib] Starting self-calibration...")

        # --- 1) compass stationary noise ---
        comp_samples = []
        t0 = time.time()
        duration = 1.0  # 1s sampling
        if verbose:
            print("[Calib] measuring compass noise for {:.1f}s...".format(duration))
        end = t0 + duration
        while time.time() < end:
            c = self._read_compass()
            if c is not None:
                comp_samples.append(c)
            time.sleep(0.05)
        if len(comp_samples) >= 3:
            # unwrap angles and compute std
            arr = np.array(comp_samples)
            # unwrap change-wise
            diffs = np.diff(np.unwrap(np.deg2rad(arr)))
            noise_deg = float(np.std(np.rad2deg(diffs)))
            bias = float(np.mean(arr))
            self.compass_noise_deg = max(0.1, noise_deg)
            if verbose:
                print(f"[Calib] compass samples: {len(arr)}, bias ~ {bias:.2f}°, noise std ~ {self.compass_noise_deg:.2f}°")
        else:
            self.compass_noise_deg = 2.5  # fallback
            if verbose:
                print("[Calib] insufficient compass samples, using fallback noise {:.2f}°".format(self.compass_noise_deg))

        # --- 2) forward encoder response ---
        # make sure we have an encoder baseline
        enc, rx = self._read_encoders()
        if enc is None:
            if verbose:
                print("[Calib] encoders not available — skipping forward calibration.")
            # keep defaults
            return

        # wait a short moment to stabilize
        time.sleep(0.1)
        enc0, rx0 = self._read_encoders()
        if enc0 is None:
            if verbose:
                print("[Calib] encoders not available at start — abort forward calibration.")
            return

        test_cmd = 30  # small forward command used to measure response
        test_time = 0.6  # seconds
        if verbose:
            print(f"[Calib] forward test: cmd={test_cmd} for {test_time:.2f}s ...")
        # issue forward
        try:
            self.robot.base.move(Forward_Velocity=int(test_cmd))
        except Exception:
            pass
        time.sleep(test_time)
        try:
            self.robot.base.move_stop()
        except Exception:
            pass
        time.sleep(0.08)  # let enc update
        enc1, rx1 = self._read_encoders()
        if enc1 is None or enc0 is None:
            if verbose:
                print("[Calib] encoder read failed after forward test.")
            return

        # encoder change (mean of left/right)
        left_delta = abs(enc1[self.left_idx] - enc0[self.left_idx])
        right_delta = abs(enc1[self.right_idx] - enc0[self.right_idx])
        mean_delta = 0.5 * (left_delta + right_delta)
        delta_per_sec = mean_delta / max(1e-6, test_time)
        # normalize to per-unit-command: delta_per_sec / test_cmd
        expected_per_unit = delta_per_sec / max(1e-6, abs(test_cmd))
        self.expected_enc_delta_per_sec_at_unit_cmd = expected_per_unit

        # set slip threshold: expect at least 30% of expected delta over slip_detection_window;
        # allow some margin; floor to sensible minimum
        expected_over_window = expected_per_unit * abs(test_cmd) * self.slip_detection_window
        computed_threshold = max(1.0, 0.35 * expected_over_window)
        self.encoder_slip_delta_threshold = computed_threshold
        # ensure slip_min_command_mag is small enough to measure
        self.slip_min_command_mag = max(6, int(abs(test_cmd) * 0.25))
        if verbose:
            print(
                f"[Calib] encoder delta mean={mean_delta:.2f} over {test_time}s -> {delta_per_sec:.2f}/s. "
                f"expected_per_unit={expected_per_unit:.6f}. "
                f"slip_threshold set to {self.encoder_slip_delta_threshold:.2f} (window {self.slip_detection_window}s)."
            )

        # --- 3) rotation response ---
        # measure compass response to a short rotation command (wheel-level)
        # prefer to use compass if available
        comp_before = self._read_compass()
        rot_cmd = 40
        rot_time = 0.5
        if comp_before is None:
            if verbose:
                print("[Calib] compass not available — skipping rotation calibration.")
            # still return; encoder calibration applied
            return

        if verbose:
            print(f"[Calib] rotation test: cmd={rot_cmd} for {rot_time:.2f}s ... (using compass)")
        # send rotation command (wheel-level)
        try:
            self.robot.base.move(Rotation_Velocity=int(rot_cmd))
        except Exception:
            pass
        time.sleep(rot_time)
        try:
            self.robot.base.move_stop()
        except Exception:
            pass
        time.sleep(0.08)
        comp_after = self._read_compass()
        if comp_after is None or comp_before is None:
            if verbose:
                print("[Calib] compass read failed after rotation test.")
            return

        # shortest signed delta
        dh = (comp_after - comp_before + 540.0) % 360.0 - 180.0
        deg_per_sec = abs(dh) / max(1e-6, rot_time)
        # normalize per rotation-unit command
        expected_rot_per_unit = deg_per_sec / max(1e-6, abs(rot_cmd))
        self.expected_deg_per_sec_at_unit_rot = expected_rot_per_unit

        if verbose:
            print(
                f"[Calib] compass rotated {dh:.2f}° in {rot_time}s -> {deg_per_sec:.2f}°/s. "
                f"expected_deg_per_unit={expected_rot_per_unit:.5f}°/s per rot-unit."
            )

        # Adjust how we detect "failed rotation" — set a minimal expected change per second for reasonable rotation commands
        # We'll use this in update_and_maybe_recover when checking rotation effectiveness
        return

    def update_and_maybe_recover(self, commanded_forward, commanded_rotation, commanded_sideways):
        now = time.time()
        enc, rx = self._read_encoders()
        compass_angle = self._read_compass()

        if self.enc_last is None and enc is not None:
            self.enc_last = enc.copy()
            self.enc_last_time = now
            self.enc_rx_last = rx
        if self.compass_last is None and compass_angle is not None:
            self.compass_last = compass_angle
            self.compass_last_time = now

        out_forward = commanded_forward
        out_rotation = commanded_rotation
        out_sideways = commanded_sideways

        # --- detect forward slip ---
        if enc is not None and self.enc_last is not None:
            dt = max(1e-6, now - self.enc_last_time)
            left_delta = abs(enc[self.left_idx] - self.enc_last[self.left_idx])
            right_delta = abs(enc[self.right_idx] - self.enc_last[self.right_idx])
            mean_delta = 0.5 * (left_delta + right_delta)

            # compute expected delta for the commanded_forward over dt if we have a calibration
            expected_over_dt = None
            if self.expected_enc_delta_per_sec_at_unit_cmd is not None:
                expected_over_dt = (
                    abs(commanded_forward) * self.expected_enc_delta_per_sec_at_unit_cmd * dt
                )

            # if commanded forward magnitude significant and mean_delta small -> slip
            min_cmd_to_check = self.slip_min_command_mag
            if abs(commanded_forward) >= min_cmd_to_check:
                # choose threshold: prefer calibrated expected_over_dt, else fallback to encoder_slip_delta_threshold
                threshold = self.encoder_slip_delta_threshold
                if expected_over_dt is not None:
                    # set threshold fraction of expected (e.g. 30%)
                    threshold = max(1.0, 0.30 * expected_over_dt)
                if mean_delta < threshold:
                    if self.slip_since is None:
                        self.slip_since = now
                    elif (now - self.slip_since) >= self.slip_detection_window and not self.slip_recovering:
                        self.slip_recovering = True
                        print("[Supervisor] wheel slip detected (mean_delta={:.2f}, threshold={:.2f}) — performing recovery".format(mean_delta, threshold))
                        self.robot.base.move_stop()
                        self.robot.base.led.setcolor(ColorLed.RED, ColorLed.LED_ON)
                        try:
                            self.robot.base.move(Forward_Velocity=int(RECOVERY_REVERSE_SPEED))
                        except Exception:
                            pass
                        time.sleep(RECOVERY_REVERSE_TIME)
                        self.robot.base.move_stop()

                        try:
                            vals = self.robot.body.distance_sensors.get_all_values() / 65536.0
                            n = len(vals)
                            left_sum = np.sum(vals[: n // 2]) if n > 1 else 1.0
                            right_sum = np.sum(vals[n // 2 :]) if n > 1 else 1.0
                            rot_sign = 1 if left_sum < right_sum else -1
                        except Exception:
                            rot_sign = 1

                        cur_heading = compass_angle if compass_angle is not None else (self.compass_last if self.compass_last is not None else 0.0)
                        target = (cur_heading + rot_sign * RECOVERY_ROTATE_DEG) % 360
                        try:
                            self.robot.body.compass.rotate_absolute(target, wait_for_finish=True, rotation_tmo_threshold=RECOVERY_ROTATE_TIMEOUT)
                        except Exception:
                            try:
                                if rot_sign > 0:
                                    self.robot.base.move(Rotation_Velocity=30)
                                else:
                                    self.robot.base.move(Rotation_Velocity=-30)
                                time.sleep(0.38)
                                self.robot.base.move_stop()
                            except Exception:
                                pass

                        self.robot.base.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
                        self.slip_recovering = False
                        self.slip_since = None
                else:
                    self.slip_since = None
            else:
                self.slip_since = None

            self.enc_last = enc.copy()
            self.enc_last_time = now
            self.enc_rx_last = rx

        # --- detect rotation slip (wheel rotation commanded but compass didn't change enough) ---
        if compass_angle is not None and self.compass_last is not None:
            dh = (compass_angle - self.compass_last + 540.0) % 360.0 - 180.0
            dt = max(1e-6, now - self.compass_last_time)

            if abs(out_rotation) >= 8:
                expected_deg_per_sec_per_unit = self.expected_deg_per_sec_at_unit_rot
                if expected_deg_per_sec_per_unit is not None:
                    expected_deg_over_dt = abs(out_rotation) * expected_deg_per_sec_per_unit * dt
                    # if observed rotation much smaller than expected (e.g. < 30%), do accurate compass rotate
                    if abs(dh) < max(0.8, 0.30 * expected_deg_over_dt):
                        print("[Supervisor] rotation did not materialize (comp delta {:.2f}°, expected {:.2f}°). Using compass rotate.".format(dh, expected_deg_over_dt))
                        target = (self.compass_last + math.copysign(max(12.0, abs(out_rotation) * 0.5), out_rotation)) % 360.0
                        try:
                            self.robot.body.compass.rotate_absolute(target, wait_for_finish=True, rotation_tmo_threshold=RECOVERY_ROTATE_TIMEOUT)
                        except Exception:
                            pass
                        out_rotation = 0
                else:
                    # we don't have a rotation calibration; fall back to noise-based check
                    if abs(dh) < max(1.0, self.compass_noise_deg * 1.5):
                        print("[Supervisor] rotation small (dh {:.2f}°) vs noise {:.2f}° — doing compass rotate".format(dh, self.compass_noise_deg))
                        target = (self.compass_last + math.copysign(20.0, out_rotation)) % 360.0
                        try:
                            self.robot.body.compass.rotate_absolute(target, wait_for_finish=True, rotation_tmo_threshold=RECOVERY_ROTATE_TIMEOUT)
                        except Exception:
                            pass
                        out_rotation = 0

            self.compass_last = compass_angle
            self.compass_last_time = now

        return int(out_forward), int(out_rotation), int(out_sideways)


# === Sensor plot update (keeps your polar plots) ===
def update_sensor_plot(ax1, ax2, robot):
    raw_values = robot.body.distance_sensors.get_all_values() / 65536.0
    sensor_angles_bottom = getattr(robot.body.distance_sensors, "sensor_angles_bottom", None)
    sensor_angles_mid = getattr(robot.body.distance_sensors, "sensor_angles_mid", None)

    ax1.clear()
    ax1.set_theta_offset(np.pi / 2)
    ax1.set_theta_direction(-1)
    ax1.set_rlim(0, 1)
    ax2.clear()
    ax2.set_theta_offset(np.pi / 2)
    ax2.set_theta_direction(-1)
    ax2.set_rlim(0, 1)

    try:
        if sensor_angles_bottom is not None and sensor_angles_mid is not None:
            for i in range(min(8, len(raw_values))):
                start_angle = sensor_angles_bottom[i, 0] / 180.0 * math.pi
                stop_angle = sensor_angles_bottom[i, 1] / 180.0 * math.pi
                color = "lightgrey"
                if raw_values[i] < 0.5:
                    color = "yellow"
                if raw_values[i] < 0.3:
                    color = "red"
                ax1.fill_between([start_angle, stop_angle], [raw_values[i], raw_values[i]], y2=1, color=color)

            for i in range(min(3, max(0, len(raw_values) - 8))):
                start_angle = sensor_angles_mid[i, 0] / 180.0 * math.pi
                stop_angle = sensor_angles_mid[i, 1] / 180.0 * math.pi
                color = "lightgrey"
                if raw_values[i + 8] < 0.5:
                    color = "yellow"
                if raw_values[i + 8] < 0.3:
                    color = "red"
                ax2.fill_between([start_angle, stop_angle], [raw_values[i + 8], raw_values[i + 8]], y2=1, color=color)
        else:
            sensor_count = len(raw_values)
            sensor_angles = np.linspace(-90.0, 90.0, sensor_count) * math.pi / 180.0
            for i, val in enumerate(raw_values):
                start_angle = sensor_angles[i] - (math.pi / sensor_count)
                stop_angle = sensor_angles[i] + (math.pi / sensor_count)
                color = "lightgrey"
                if val < 0.5:
                    color = "yellow"
                if val < 0.3:
                    color = "red"
                ax1.fill_between([start_angle, stop_angle], val, 1, color=color)
    except Exception:
        pass

    return np.asarray(raw_values, dtype=float)


# === Main autonomous loop ===
def main():
    fig, (ax1, ax2) = plt.subplots(1, 2, subplot_kw={"projection": "polar"}, figsize=(12, 6))
    plt.ion()
    plt.show()
    ax1.set_rlim(0, 1)
    ax2.set_rlim(0, 1)

    robot = SaraRobot(logging=False)
    time.sleep(1.0)

    avoidance = SmartAvoidance(
        base_diameter_cm=BASE_DIAMETER_CM,
        sensor_count=SENSOR_COUNT,
        sensor_range_cm=SENSOR_RANGE_CM,
        min_object_width_cm=MIN_OBJECT_WIDTH_CM,
        passable_margin_cm=PASSABLE_MARGIN_CM,
        memory_decay=0.986,
        sensitivity=0.30,
        side_gain=16.0,
        rot_gain=14.0,
        forward_scale=28,
        memory_weight=0.32,
    )

    supervisor = MotionSupervisor(robot)

    # === calibration step before starting autonomy ===
    try:
        supervisor.calibrate(verbose=True)
    except Exception as e:
        print("[Calib] calibration error:", e)

    autonomous_mode = True
    print("Autonomous mode ON. Press '3' to toggle manual/autonomous. ESC to quit.")

    old_counter = robot.body.distance_sensors.get_rx_counter()
    last_plot = time.time()

    try:
        while True:
            # Read sensors & update plots (this returns normalized 0..1)
            sensor_values = update_sensor_plot(ax1, ax2, robot)

            # Toggle autonomous/manual
            if keyboard.is_pressed("3"):
                autonomous_mode = not autonomous_mode
                print("Autonomous mode:", "ON" if autonomous_mode else "OFF")
                time.sleep(0.4)

            if autonomous_mode:
                forward_v, rotation_v, sideways_v = avoidance.compute(sensor_values)

                # pass commanded velocities through supervisor which may modify them if slip detected
                safe_forward, safe_rotation, safe_sideways = supervisor.update_and_maybe_recover(forward_v, rotation_v, sideways_v)

                # LED feedback
                if np.min(sensor_values) < 0.18:
                    robot.base.led.setcolor(ColorLed.RED, ColorLed.LED_ON)
                else:
                    robot.base.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)

                # Send integers to robot — Sara SDK expects integers for bitwise ops
                robot.base.move(
                    Sideways_Velocity=int(safe_sideways),
                    Forward_Velocity=int(safe_forward),
                    Rotation_Velocity=int(safe_rotation),
                )

            else:
                forward = 0
                sideways = 0
                rotation = 0
                speed = 50
                rotation_speed = 60

                if keyboard.is_pressed("w"):
                    forward = speed
                elif keyboard.is_pressed("s"):
                    forward = -speed
                if keyboard.is_pressed("d"):
                    rotation = -rotation_speed
                elif keyboard.is_pressed("a"):
                    rotation = rotation_speed
                if keyboard.is_pressed("q"):
                    sideways = -speed
                elif keyboard.is_pressed("e"):
                    sideways = speed

                if not any([forward, sideways, rotation]):
                    robot.base.move_stop()
                else:
                    robot.base.move(
                        Sideways_Velocity=int(sideways),
                        Forward_Velocity=int(forward),
                        Rotation_Velocity=int(rotation),
                    )

            # update matplotlib occasionally (reduce CPU)
            if time.time() - last_plot > 0.06:
                try:
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                except Exception:
                    pass
                last_plot = time.time()

            # exit if ESC pressed
            if keyboard.is_pressed("esc"):
                print("ESC pressed: exiting.")
                break

            # wait for next sensor packet (avoid busy-loop)
            while old_counter == robot.body.distance_sensors.get_rx_counter():
                time.sleep(0.02)
            old_counter = robot.body.distance_sensors.get_rx_counter()

    except KeyboardInterrupt:
        pass
    finally:
        robot.base.move_stop()
        robot.base.led.setcolor(ColorLed.WHITE, ColorLed.LED_ON)
        robot.base.brake(ApplyBrake=False)
        robot.stop()
        plt.close(fig)


if __name__ == "__main__":
    main()
