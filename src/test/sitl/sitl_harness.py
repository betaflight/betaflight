#!/usr/bin/env python3
"""SITL end-to-end harness: UDP RC + FDM feeds, MSP introspection, scenario runner.

Drives a betaflight_SITL binary the way a simulator and transmitter would:
  - RC channels over UDP :9004 (rc_packet: double timestamp + 16 x uint16)
  - FDM state over UDP :9003 (fdm_packet: 18 doubles; virtual-GPS mode puts
    lon/lat/alt in position_xyz and ENU velocity in velocity_xyz)
  - MSP over TCP :5761 for runtime state (modes, arming disable flags)
  - one-shot `--config <file>` runs to provision eeprom.bin per scenario

Scenarios exercise the flight plan / AUTOPILOT safety behaviour end to end:
mode wiring, rx-loss policies (DISABLE / CONTINUE / LAND) and geofence
(LAND / RTH). Requires a SITL binary built with USE_FLIGHT_PLAN.

Usage:
  sitl_harness.py --binary obj/main/betaflight_SITL.elf --scenario all
  sitl_harness.py --binary ... --scenario rx_continue -v
"""

import argparse
import math
import os
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time

MSP_STATUS = 101
MSP_RAW_GPS = 106
MSP_BOXIDS = 119
MSP_ACC_CALIBRATION = 205

TCP_PORT = 5761
RC_PORT = 9004
FDM_PORT = 9003
PWM_PORT = 9002

HOME_LAT = -27.5000000
HOME_LON = 153.0000000
HOME_ALT_M = 30.0
M_PER_DEG = 111319.49

# Box permanent IDs (msp_box.c)
BOX_ARM = 0
BOX_ALTHOLD = 3
BOX_POSHOLD = 11
BOX_FAILSAFE = 27
BOX_GPSRESCUE = 46
BOX_AUTOPILOT = 56

RC_MID = 1500
RC_LOW = 1000
RC_HIGH = 2000

VERBOSE = False


def log(msg):
    print(f"[harness] {msg}", flush=True)


def debug(msg):
    if VERBOSE:
        log(msg)


class RcFeed(threading.Thread):
    """50 Hz rc_packet stream. Stop the stream to simulate RX loss."""

    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.channels = [RC_MID, RC_MID, RC_LOW, RC_MID] + [RC_LOW] * 12  # AERT + AUX
        self.streaming = True
        self.running = True
        self.t0 = time.monotonic()

    def set(self, index, value):
        self.channels[index] = value

    def run(self):
        while self.running:
            if self.streaming:
                pkt = struct.pack("<d16H", time.monotonic() - self.t0, *self.channels)
                self.sock.sendto(pkt, ("127.0.0.1", RC_PORT))
            time.sleep(0.02)

    def stop_stream(self):
        self.streaming = False

    def shutdown(self):
        self.running = False


class MotorFeed(threading.Thread):
    """Listens for SITL's normalised motor outputs (servo_packet on UDP 9002)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", PWM_PORT))
        self.sock.settimeout(0.2)
        self.motors = [0.0, 0.0, 0.0, 0.0]
        self.running = True

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(64)
                if len(data) >= 16:
                    self.motors = list(struct.unpack("<4f", data[:16]))
            except socket.timeout:
                pass
            except OSError:
                break  # socket closed during shutdown

    def shutdown(self):
        # release the port 9002 bind before the next scenario constructs its feed
        self.running = False
        if self.is_alive():
            self.join(timeout=1.0)
        self.sock.close()


GRAVITY = 9.80665
HOVER_THRUST = 0.30        # just above the FC hoverThrottle default (1275 -> 0.275)
RATE_GAIN = 12.0           # rad/s of body rate per unit differential thrust
RATE_TAU = 0.08            # attitude response time constant, seconds
# Deliberately draggy (a real 5-inch holds 5 m/s at ~20 deg, not ~50): the
# nav position controller tracks its velocity target through an integrator
# and oscillates against a low-drag plant until the velocity-mode loop
# (ap_velocity_*, currently unconsumed) exists.
K_DRAG = 2.4               # 1/s, linear drag: 50 deg of tilt holds ~4.9 m/s
VERT_V_GAIN = 6.0          # terminal climb rate per unit thrust above hover
VEL_TAU_V = 0.3            # vertical velocity response, seconds


class MotionModel:
    """Crude quad kinematics: motor outputs -> body rates -> attitude -> motion.

    Differential thrust maps to first-order body-rate targets (quad-X, BF motor
    order M1=RR M2=FR M3=RL M4=FL), collective maps to thrust along body Z.
    Just enough plant for Betaflight's real rate/angle/position loops to close
    around; not a physics simulation.
    """

    def __init__(self):
        self.pos = [0.0, 0.0, 0.0]   # ENU metres relative to home (ground = 0 up)
        self.vel = [0.0, 0.0, 0.0]   # ENU m/s
        self.accel = [0.0, 0.0, 0.0]  # ENU m/s^2 (world frame, for the acc feed)
        self.roll = 0.0              # rad, right positive
        self.pitch = 0.0             # rad, nose-down positive (BF command convention)
        self.yaw = 0.0               # rad, compass (CW from north) positive
        self.rates = [0.0, 0.0, 0.0]
        self.impact_ticks = 0

    def on_ground(self):
        return self.pos[2] <= 0.001

    def step(self, dt, m):
        thrust = sum(m) / 4.0

        if self.on_ground() and thrust < HOVER_THRUST * 0.8:
            self.pos[2] = 0.0
            self.vel = [0.0, 0.0, 0.0]
            # touchdown impact: a short accelerometer spike, as a real landing
            # produces, so the FC's jerk-based disarmOnImpact can trigger
            self.accel = [0.0, 0.0, 60.0] if self.impact_ticks > 0 else [0.0, 0.0, 0.0]
            self.impact_ticks = max(0, self.impact_ticks - 1)
            self.rates = [0.0, 0.0, 0.0]
            return

        right = m[0] + m[1]   # M1 RR + M2 FR
        left = m[2] + m[3]    # M3 RL + M4 FL
        rear = m[0] + m[2]
        front = m[1] + m[3]
        # BF quad-X defaults: M1/M4 spin CW, M2/M3 CCW; reaction torque yaws
        # the frame opposite the prop direction.
        ccw = m[1] + m[2]
        cw = m[0] + m[3]

        target = [
            RATE_GAIN * (left - right) / 2.0,   # roll right
            RATE_GAIN * (rear - front) / 2.0,   # nose down (BF mixer: +pitch = rear up)
            RATE_GAIN * (ccw - cw) / 2.0,       # yaw CW (compass positive)
        ]
        for i in range(3):
            self.rates[i] += (target[i] - self.rates[i]) * min(1.0, dt / RATE_TAU)
        self.roll += self.rates[0] * dt
        self.pitch += self.rates[1] * dt
        self.yaw += self.rates[2] * dt
        self.roll = max(-1.2, min(1.2, self.roll))
        self.pitch = max(-1.2, min(1.2, self.pitch))

        # Horizontal: thrust-vector acceleration with linear drag
        # (dv/dt = g*tan(tilt) - K_DRAG*v). Vertical: first-order response
        # toward the thrust-above-hover terminal rate, the plant shape
        # alt-hold is tuned for; tilt reduces the vertical thrust component
        # (the FC compensates with 1/cos(tilt); the plant must charge for it
        # or that boost becomes a climb bias during fast forward flight).
        a_fwd = GRAVITY * math.tan(self.pitch)
        a_right = GRAVITY * math.tan(self.roll)
        sin_y, cos_y = math.sin(self.yaw), math.cos(self.yaw)
        acc_h = [
            a_fwd * sin_y + a_right * cos_y - K_DRAG * self.vel[0],  # east
            a_fwd * cos_y - a_right * sin_y - K_DRAG * self.vel[1],  # north
        ]
        for i in range(2):
            self.accel[i] = acc_h[i]
            self.vel[i] += acc_h[i] * dt
            self.pos[i] += self.vel[i] * dt

        vt_z = VERT_V_GAIN * (thrust * math.cos(self.pitch) * math.cos(self.roll) - HOVER_THRUST)
        new_vz = self.vel[2] + (vt_z - self.vel[2]) * min(1.0, dt / VEL_TAU_V)
        self.accel[2] = (new_vz - self.vel[2]) / dt if dt > 0 else 0.0
        self.vel[2] = new_vz
        self.pos[2] += self.vel[2] * dt

        if self.pos[2] < 0.0:
            self.pos[2] = 0.0
            self.vel = [0.0, 0.0, 0.0]
            self.accel = [0.0, 0.0, 0.0]
            self.rates = [0.0, 0.0, 0.0]
            self.roll = self.pitch = 0.0
            self.impact_ticks = 4


def quat_from_euler_bf(roll, pitch, yaw):
    """Body->world quaternion in Betaflight's internal NWU frames from the
    model conventions (roll right+, pitch nose-down+, yaw compass CW+):
    NWU yaw is CCW-positive, pitch and roll map directly."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(-yaw / 2), math.sin(-yaw / 2)
    return (
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    )


def quat_conj_x180(q):
    """Similarity transform by 180 deg about x: what the gazebo plugin applies
    to its quaternion, and what the FC's bridge undoes on receive."""
    return (q[0], q[1], -q[2], -q[3])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_rotate_inv(q, v):
    """Rotate world vector v into the body frame (q is body->world)."""
    qc = (q[0], -q[1], -q[2], -q[3])
    p = quat_mul(quat_mul(qc, (0.0, *v)), q)
    return (p[1], p[2], p[3])


K_RZ_NEG90 = (math.sqrt(0.5), 0.0, 0.0, -math.sqrt(0.5))


class FdmFeed(threading.Thread):
    """50 Hz fdm_packet stream driven by the motion model.

    Emits in the Gazebo-bridge conventions the default SITL build expects:
    quaternion pre-multiplied by Rz(-90deg) (the FC re-applies Rz(+90deg)),
    gyro in the plugin sensor frame (pitch and yaw negated from the model's
    nose-down/compass-CW conventions), and GPS lat/lon mirrored around the
    first packet's origin (the FC un-mirrors).
    """

    def __init__(self, motors=None):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.model = MotionModel()
        self.motors = motors
        self.running = True
        self.t0 = time.monotonic()

    def move_east(self, metres):
        self.model.pos[0] += metres

    def distance_from_home(self):
        return math.hypot(self.model.pos[0], self.model.pos[1])

    def distance_to_wp(self, east_m, north_m):
        return math.hypot(self.model.pos[0] - east_m, self.model.pos[1] - north_m)

    def heading_deg(self):
        return math.degrees(self.model.yaw) % 360.0

    def run(self):
        last = time.monotonic()
        while self.running:
            now = time.monotonic()
            dt = min(0.1, now - last)
            last = now
            m = self.motors.motors if self.motors else [0.0] * 4
            self.model.step(dt, m)

            lat_true = HOME_LAT + self.model.pos[1] / M_PER_DEG
            lon_true = HOME_LON + self.model.pos[0] / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))

            # The FC's bridge computes q = Rz(+90) * Rx(180) * q_packet * Rx(180),
            # so emit the true NWU attitude pre-rotated by Rz(-90) and
            # pre-conjugated: the FC recovers exactly q_nwu.
            q_nwu = quat_from_euler_bf(self.model.roll, self.model.pitch, self.model.yaw)
            q = quat_conj_x180(quat_mul(K_RZ_NEG90, q_nwu))

            # Specific force in the FC's earth frame (NWU), rotated into the
            # body with the same attitude the FC reconstructs, so the
            # estimator's tilt-compensation inverts this rotation exactly at
            # any heading. The packet carries the negated body vector - the
            # SITL acc driver negates all three axes on read.
            f_world_nwu = (
                self.model.accel[1],                     # north
                -self.model.accel[0],                    # west
                self.model.accel[2] + GRAVITY,           # up
            )
            f_body = quat_rotate_inv(q_nwu, f_world_nwu)

            pkt = struct.pack(
                "<18d",
                now - self.t0,
                # Gazebo-plugin gyro frame: roll right +, pitch nose-up +,
                # yaw CCW +. The model keeps nose-down/CW positive (compass
                # conventions), so pitch and yaw are negated on emit.
                self.model.rates[0], -self.model.rates[1], -self.model.rates[2],
                -f_body[0], -f_body[1], -f_body[2],                              # negated NWU-body specific force
                q[0], q[1], q[2], q[3],
                self.model.vel[0], self.model.vel[1], self.model.vel[2],         # ENU m/s
                2.0 * HOME_LON - lon_true,                                       # mirrored for the bridge
                2.0 * HOME_LAT - lat_true,
                HOME_ALT_M + self.model.pos[2],
                101325.0,
            )
            self.sock.sendto(pkt, ("127.0.0.1", FDM_PORT))
            time.sleep(0.02)

    def shutdown(self):
        self.running = False


class Msp:
    """Minimal MSP v1 client over the SITL TCP port."""

    def __init__(self, sock):
        self.sock = sock
        self.buf = b""

    def request(self, cmd, payload=b"", timeout=2.0):
        frame = struct.pack("<BB", len(payload), cmd) + payload
        checksum = 0
        for b in frame:
            checksum ^= b
        self.sock.sendall(b"$M<" + frame + bytes([checksum]))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            reply = self._read_frame(cmd, deadline)
            if reply is not None:
                return reply
        raise TimeoutError(f"no MSP reply for cmd {cmd}")

    def _read_frame(self, want_cmd, deadline):
        while time.monotonic() < deadline:
            start = self.buf.find(b"$M>")
            if start < 0:
                # also tolerate error frames
                if self.buf.find(b"$M!") >= 0:
                    raise RuntimeError(f"MSP error frame for cmd {want_cmd}")
                self._fill(deadline)
                continue
            if len(self.buf) < start + 5:
                self._fill(deadline)
                continue
            size = self.buf[start + 3]
            cmd = self.buf[start + 4]
            end = start + 5 + size + 1
            if len(self.buf) < end:
                self._fill(deadline)
                continue
            payload = self.buf[start + 5 : start + 5 + size]
            self.buf = self.buf[end:]
            if cmd == want_cmd:
                return payload
        return None

    def _fill(self, deadline):
        self.sock.settimeout(max(0.05, deadline - time.monotonic()))
        try:
            data = self.sock.recv(4096)
            if data:
                self.buf += data
        except socket.timeout:
            pass


class Sitl:
    def __init__(self, binary, workdir):
        self.binary = os.path.abspath(binary)
        self.workdir = workdir
        self.proc = None
        self.sock = None
        self.msp = None
        self.boxids = []

    def provision(self, cli_lines):
        cfg = os.path.join(self.workdir, "scenario_config.txt")
        with open(cfg, "w") as f:
            f.write("\n".join(cli_lines) + "\n")
        eeprom = os.path.join(self.workdir, "eeprom.bin")
        if os.path.exists(eeprom):
            os.remove(eeprom)
        res = subprocess.run(
            [self.binary, "--config", cfg],
            cwd=self.workdir,
            capture_output=True,
            text=True,
            timeout=60,
        )
        debug(f"provision rc={res.returncode}")
        if res.returncode != 0:
            raise RuntimeError(f"provisioning failed (rc={res.returncode}):\n{res.stdout}\n{res.stderr}")
        if not os.path.exists(eeprom):
            raise RuntimeError(f"provisioning produced no eeprom.bin:\n{res.stdout}\n{res.stderr}")

    @staticmethod
    def wait_port_free(timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                probe = socket.create_connection(("127.0.0.1", TCP_PORT), timeout=0.3)
                probe.close()
                time.sleep(0.3)
            except OSError:
                return
        raise RuntimeError("previous SITL still holds the MSP port")

    def start(self):
        # The TCP listener has no SO_REUSEADDR; sockets from a previous scenario
        # lingering in TIME_WAIT can make the bind fail silently, so retry the
        # whole launch a few times rather than only polling for the port.
        for attempt in range(3):
            self.wait_port_free()
            logf = open(os.path.join(self.workdir, "sitl.log"), "a")
            self.proc = subprocess.Popen(["stdbuf", "-oL", self.binary], cwd=self.workdir, stdout=logf, stderr=logf)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if self.proc.poll() is not None:
                    debug(f"SITL exited early (rc={self.proc.returncode}); relaunching")
                    break
                try:
                    self.sock = socket.create_connection(("127.0.0.1", TCP_PORT), timeout=1)
                    self.msp = Msp(self.sock)
                    self.boxids = list(self.msp.request(MSP_BOXIDS))
                    debug(f"boxids: {self.boxids}")
                    return
                except (OSError, TimeoutError, RuntimeError) as exc:
                    debug(f"MSP startup probe failed: {exc}")
                    time.sleep(0.3)
            debug(f"launch attempt {attempt + 1} failed; relaunching")
            self.stop()
            time.sleep(2.0)
        raise RuntimeError("SITL did not open the MSP port after 3 launches")

    def status(self):
        p = self.msp.request(MSP_STATUS)
        mode_flags = struct.unpack_from("<I", p, 6)[0]
        extra_count = p[15]
        off = 16 + extra_count
        arming_count = p[off]
        arming_flags = struct.unpack_from("<I", p, off + 1)[0]
        active = {self.boxids[i] for i in range(min(32, len(self.boxids))) if mode_flags & (1 << i)}
        return {"modes": active, "arming_flags": arming_flags, "arming_count": arming_count}

    def modes(self):
        return self.status()["modes"]

    def gps(self):
        p = self.msp.request(MSP_RAW_GPS)
        lat, lon = struct.unpack_from("<ii", p, 2)
        return {"lat": lat / 1e7, "lon": lon / 1e7}

    def distance_to_m(self, lat, lon):
        g = self.gps()
        dn = (g["lat"] - lat) * M_PER_DEG
        de = (g["lon"] - lon) * M_PER_DEG * math.cos(math.radians(lat))
        return math.hypot(dn, de)

    def acc_calibrate(self):
        self.msp.request(MSP_ACC_CALIBRATION)

    def stop(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None


def wait_for(description, predicate, timeout=20.0, interval=0.2):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = predicate()
        if last:
            log(f"ok: {description}")
            return last
        time.sleep(interval)
    raise AssertionError(f"timeout waiting for: {description}")


WP_LAT = HOME_LAT + 300.0 / M_PER_DEG  # default waypoint 300 m north of home
WP_EAST_LON = HOME_LON + 150.0 / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))  # 150 m east
WP_NORTH40_LAT = HOME_LAT + 40.0 / M_PER_DEG  # short leg for the landing mission
WP_EAST25_LON = HOME_LON + 25.0 / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))


def base_config(extra):
    return [
        "feature GPS",
        "set gps_provider = VIRTUAL",
        "set failsafe_procedure = AUTO-LAND",
        "set failsafe_delay = 10",
        "set small_angle = 180",
        "aux 0 0 0 1700 2100 0 0",   # ARM on AUX1
        "aux 1 56 1 1700 2100 0 0",  # AUTOPILOT on AUX2
        "aux 2 1 2 1700 2100 0 0",   # ANGLE on AUX3 (heading-validation flight)
        # 10 m above home, 5 m/s — low and quick keeps landing scenarios short
        f"waypoint insert 0 {WP_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 orbit",
    ] + extra


def boot_and_engage(sitl, rc, fdm):
    """Common preamble: boot, GPS fix, arm, raise throttle, engage AUTOPILOT."""
    rc.start()
    fdm.start()

    wait_for("GPS fix + RX recovery (arming flags clear)", lambda: sitl.status()["arming_flags"] == 0, timeout=40)

    # Recalibrate the accelerometer now the FDM feed is live: the boot-time
    # calibration can capture offsets from a not-yet-settled feed, and the
    # resulting bias integrates into a phantom vertical velocity.
    sitl.acc_calibrate()
    time.sleep(2.0)
    wait_for("recalibration complete", lambda: sitl.status()["arming_flags"] == 0, timeout=20)

    rc.set(6, RC_HIGH)  # AUX3: ANGLE for the manual segment
    # A transient arming-disable (e.g. an RXLOSS blip) at the moment the switch
    # goes high latches ARM_SWITCH until the switch is cycled; retry the arm.
    for attempt in range(3):
        rc.set(4, RC_HIGH)  # AUX1: arm (throttle is low)
        try:
            wait_for("armed", lambda: BOX_ARM in sitl.modes(), timeout=8)
            break
        except AssertionError:
            if attempt == 2:
                raise
            log("arm attempt latched ARM_SWITCH; cycling the switch")
            rc.set(4, 1000)
            time.sleep(1.0)

    rc.set(2, 1600)     # raise throttle (wasThrottleRaised) and climb clear of the ground
    time.sleep(3.0)

    rc.set(5, RC_HIGH)  # AUX2: AUTOPILOT
    required_modes = {BOX_AUTOPILOT, BOX_ALTHOLD, BOX_POSHOLD}

    def active_required_modes():
        modes = sitl.modes()
        return modes if modes >= required_modes else None

    modes = wait_for(
        "AUTOPILOT + ALTHOLD + POSHOLD active (mode wiring)",
        active_required_modes,
    )
    rc.set(2, 1300)     # throttle into the alt-hold deadband: no stick adjustments
    return modes


def scenario_mission_flight(sitl, rc, fdm):
    """Closed-loop flight: the mission leg is actually flown by the motion
    model under Betaflight's own controllers, ending parked at the waypoint."""
    boot_and_engage(sitl, rc, fdm)

    wait_for(
        "vehicle departs toward the waypoint (>15 m from home)",
        lambda: fdm.distance_from_home() > 15.0,
        timeout=30,
    )
    # Assertions use the model's ground truth; the FC estimator tracks it,
    # while MSP_RAW_GPS leads the true position (virtual-GPS extrapolation).
    wait_for(
        "waypoint reached (within 8 m, ground truth)",
        lambda: fdm.distance_to_wp(0.0, 300.0) < 8.0,
        timeout=150,
        interval=1.0,
    )
    # Mission complete: executor parks in position hold at the waypoint.
    # Legs complete on radius entry; the hold-mode braking parks a short
    # distance past the point at cruise speed.
    wait_for(
        "settled near the waypoint",
        lambda: math.hypot(fdm.model.vel[0], fdm.model.vel[1]) < 1.0 and fdm.distance_to_wp(0.0, 300.0) < 25.0,
        timeout=60,
        interval=2.0,
    )
    # dwell: a transit averages cruise speed, a hold oscillates about the
    # point (instantaneous peaks reach ~2 m/s with SITL's 15 Hz position loop)
    samples = []
    for _ in range(10):
        time.sleep(1.0)
        samples.append(math.hypot(fdm.model.vel[0], fdm.model.vel[1]))
    dist = fdm.distance_to_wp(0.0, 300.0)
    avg_speed = sum(samples) / len(samples)
    assert dist < 25.0, f"did not hold position near waypoint: {dist:.1f} m away"
    assert avg_speed < 1.5, f"did not settle at waypoint: averaging {avg_speed:.1f} m/s"
    assert BOX_ARM in sitl.modes(), "unexpected disarm at mission end"
    log(f"parked {dist:.1f} m from the waypoint")


def scenario_mission_yaw(sitl, rc, fdm):
    """Default VELOCITY yaw mode: flying an eastbound leg, the nose must swing
    from north to the ground course and hold it while the leg is flown."""
    boot_and_engage(sitl, rc, fdm)

    wait_for(
        "vehicle departs east toward the waypoint",
        lambda: fdm.model.pos[0] > 15.0,
        timeout=30,
    )

    def on_course():
        err = (fdm.heading_deg() - 90.0 + 180.0) % 360.0 - 180.0
        return abs(err) < 25.0

    wait_for("nose tracks the course (heading ~090)", on_course, timeout=30)
    time.sleep(3.0)
    assert on_course(), f"heading did not hold the course: {fdm.heading_deg():.0f} deg"

    # SITL's starved LOW-priority scheduler runs the position controller at
    # ~15-20 Hz (vs 100 Hz on hardware), so the braking phase can wander
    # before converging; the timeout allows for it.
    wait_for(
        "waypoint reached (ground truth)",
        lambda: fdm.distance_to_wp(150.0, 0.0) < 10.0,
        timeout=90,
        interval=1.0,
    )
    assert BOX_ARM in sitl.modes(), "unexpected disarm during yaw mission"
    log(f"leg flown nose-first, heading {fdm.heading_deg():.0f} deg at arrival")


def scenario_mission_land(sitl, rc, fdm):
    """LAND waypoint: fly the first leg north, divert to the offset LAND
    waypoint, arrive through the 3D hold gate, descend there, and disarm on
    touchdown (impact jerk; the estimator's vz is unreliable when grounded)."""
    boot_and_engage(sitl, rc, fdm)

    wait_for(
        "vehicle departs toward the first waypoint",
        lambda: fdm.distance_from_home() > 15.0,
        timeout=30,
    )
    wait_for(
        "LAND waypoint approach (ground truth)",
        lambda: fdm.distance_to_wp(25.0, 40.0) < 6.0,
        timeout=120,
        interval=1.0,
    )
    wait_for(
        "touchdown disarms",
        lambda: BOX_ARM not in sitl.modes(),
        timeout=90,
        interval=1.0,
    )
    assert fdm.model.on_ground(), f"disarmed in the air: alt {fdm.model.pos[2]:.1f} m"
    dist = fdm.distance_to_wp(25.0, 40.0)
    assert dist < 10.0, f"landed {dist:.1f} m from the LAND waypoint"
    log(f"landed {dist:.1f} m from the LAND waypoint")


def scenario_rx_loss(sitl, rc, fdm, policy):
    boot_and_engage(sitl, rc, fdm)
    log(f"killing RC stream (policy={policy})")
    rc.stop_stream()

    if policy == "CONTINUE":
        wait_for(
            "failsafe active with mission continuing",
            lambda: {BOX_FAILSAFE, BOX_AUTOPILOT} <= sitl.modes(),
        )
        time.sleep(3)
        modes = sitl.modes()
        assert {BOX_FAILSAFE, BOX_AUTOPILOT} <= modes, f"CONTINUE state did not persist: {modes}"
        log("mission still flying 3 s into failsafe")
    elif policy == "LAND":
        wait_for(
            "failsafe landing with mission disengaged",
            lambda: (lambda m: BOX_FAILSAFE in m and BOX_AUTOPILOT not in m
                     and {BOX_ALTHOLD, BOX_POSHOLD} <= m)(sitl.modes()),
        )
    else:  # DISABLE
        wait_for(
            "failsafe active with mission disengaged",
            lambda: (lambda m: BOX_FAILSAFE in m and BOX_AUTOPILOT not in m)(sitl.modes()),
        )


def scenario_geofence(sitl, rc, fdm, action):
    boot_and_engage(sitl, rc, fdm)
    log(f"flying north until the 50 m geofence trips (action={action})")
    # The FC's gpsSol leads ground truth, so the fence fires around 40-50 m of
    # true distance; the resulting action is the observable, not the distance.
    wait_for("mission departs toward the fence", lambda: fdm.distance_from_home() > 25.0, timeout=60)

    if action == "RTH":
        wait_for(
            "GPS rescue engaged, mission disengaged",
            lambda: (lambda m: BOX_GPSRESCUE in m and BOX_AUTOPILOT not in m)(sitl.modes()),
            timeout=60,
        )
        time.sleep(3)
        modes = sitl.modes()
        assert BOX_GPSRESCUE in modes and BOX_AUTOPILOT not in modes, \
            f"geofence rescue did not remain latched: {modes}"
        log("rescue remains latched")
    else:  # LAND
        time.sleep(8)
        modes = sitl.modes()
        if BOX_ARM in modes:
            assert BOX_AUTOPILOT in modes, f"mission dropped instead of landing: {modes}"
        assert BOX_GPSRESCUE not in modes, f"unexpected rescue: {modes}"
        log("mission holds LANDING state at current position")
        # The motion model descends under the landing command until ground
        # contact; the touchdown detector must then disarm.
        wait_for(
            "touchdown detection disarms",
            lambda: BOX_ARM not in sitl.modes(),
            timeout=45,
            interval=1.0,
        )


def scenario_geofence_rth_rxloss(sitl, rc, fdm):
    """The RTH latch must survive RX loss: stage-2 rxfail values force the
    AUTOPILOT switch low (rxfail preset in this scenario's config), which must
    not read as the pilot cancelling the rescue."""
    boot_and_engage(sitl, rc, fdm)
    wait_for(
        "GPS rescue engaged after geofence breach",
        lambda: BOX_GPSRESCUE in sitl.modes(),
        timeout=60,
    )

    log("killing RC stream mid-rescue")
    rc.stop_stream()
    wait_for("failsafe active", lambda: BOX_FAILSAFE in sitl.modes())
    time.sleep(3)
    modes = sitl.modes()
    assert {BOX_FAILSAFE, BOX_GPSRESCUE} <= modes, f"rescue dropped when rxfail cleared the switch: {modes}"
    log("rescue persists through RX loss")


SCENARIOS = {
    "baseline": (lambda s, r, f: boot_and_engage(s, r, f), []),
    # FIXED yaw: this scenario validates pure translation control; yaw-coupled
    # flight is mission_yaw's job (SITL's 15-20 Hz position loop wanders under
    # the default VELOCITY yaw during braking)
    "mission_flight": (scenario_mission_flight, ["set ap_yaw_mode = FIXED"]),
    "mission_yaw": (
        scenario_mission_yaw,
        # redirect the default waypoint east so the course demands a 90 deg swing
        [f"waypoint update 0 {HOME_LAT:.7f} {WP_EAST_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 orbit"],
    ),
    "mission_land": (
        scenario_mission_land,
        [
            "set ap_yaw_mode = FIXED",
            # short north leg, then LAND at an offset so the executor must fly
            # to the landing point before descending
            f"waypoint update 0 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 orbit",
            f"waypoint insert 1 {WP_NORTH40_LAT:.7f} {WP_EAST25_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 land 0 orbit",
            # SITL's 15-20 Hz position loop wanders during braking; a fatter
            # 3D gate keeps arrival deterministic
            "set ap_waypoint_hold_radius = 400",
            "set ap_landing_descent_rate = 200",  # keep the descent short
            "set landing_disarm_threshold = 10",   # jerk-based touchdown disarm
        ],
    ),
    "rx_disable": (lambda s, r, f: scenario_rx_loss(s, r, f, "DISABLE"), ["set ap_rx_loss_policy = DISABLE"]),
    "rx_continue": (lambda s, r, f: scenario_rx_loss(s, r, f, "CONTINUE"), ["set ap_rx_loss_policy = CONTINUE"]),
    "rx_land": (lambda s, r, f: scenario_rx_loss(s, r, f, "LAND"), ["set ap_rx_loss_policy = LAND"]),
    "geofence_land": (
        lambda s, r, f: scenario_geofence(s, r, f, "LAND"),
        [
            "set ap_max_distance_from_home = 50",
            "set ap_geofence_action = LAND",
            "set ap_landing_descent_rate = 200",  # keep the descent short
            "set landing_disarm_threshold = 10",   # jerk-based touchdown disarm
        ],
    ),
    "geofence_rth": (
        lambda s, r, f: scenario_geofence(s, r, f, "RTH"),
        ["set ap_max_distance_from_home = 50", "set ap_geofence_action = RTH"],
    ),
    "geofence_rth_rxloss": (
        scenario_geofence_rth_rxloss,
        [
            "set ap_max_distance_from_home = 50",
            "set ap_geofence_action = RTH",
            "rxfail 5 s 1000",  # stage 2 forces the AUTOPILOT switch low
        ],
    ),
}


def run_scenario(name, binary, workdir):
    body, extra_cfg = SCENARIOS[name]
    scenario_dir = os.path.join(workdir, name)
    shutil.rmtree(scenario_dir, ignore_errors=True)
    os.makedirs(scenario_dir)

    log(f"=== scenario: {name}")
    sitl = Sitl(binary, scenario_dir)
    rc = motors = fdm = None
    try:
        # feed construction can fail (port 9002 bind); it must fail the
        # scenario, not abort the suite
        rc = RcFeed()
        motors = MotorFeed()
        fdm = FdmFeed(motors)
        sitl.provision(base_config(extra_cfg))
        sitl.start()
        motors.start()
        body(sitl, rc, fdm)
        log(f"=== PASS: {name}")
        return True
    except (AssertionError, RuntimeError, TimeoutError, OSError) as e:
        log(f"=== FAIL: {name}: {e}")
        return False
    finally:
        for feed in (rc, fdm, motors):
            if feed is not None:
                feed.shutdown()
        sitl.stop()


def main():
    global VERBOSE
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--binary", required=True, help="path to betaflight_SITL.elf (built with USE_FLIGHT_PLAN)")
    ap.add_argument("--scenario", default="all", choices=["all"] + list(SCENARIOS))
    ap.add_argument("--workdir", default="/tmp/sitl_harness")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    VERBOSE = args.verbose

    os.makedirs(args.workdir, exist_ok=True)
    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    results = {name: run_scenario(name, args.binary, args.workdir) for name in names}

    log("--- summary")
    for name, ok in results.items():
        log(f"{'PASS' if ok else 'FAIL'}  {name}")
    sys.exit(0 if all(results.values()) else 1)


if __name__ == "__main__":
    main()
