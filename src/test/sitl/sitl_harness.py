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
import collections
import itertools
import json
import math
import os
import random
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import uuid

MSP_STATUS = 101
MSP_RAW_GPS = 106
MSP_ATTITUDE = 108
MSP_BOXIDS = 119
MSP_ACC_CALIBRATION = 205
MSP_DEBUG = 254

TCP_PORT = 5761
RC_PORT = 9004
FDM_PORT = 9003
PWM_PORT = 9002
PWM_RAW_PORT = 9001

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
BOX_LAUNCH = 58

RC_MID = 1500
RC_LOW = 1000
RC_HIGH = 2000

VERBOSE = False
TELEMETRY_PORT = 9005  # ground-truth JSON fan-out for external visualisers, 0 disables


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
        self.trace = None         # a list collects (time, motor 0) for every packet
        self.running = True

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(64)
                if len(data) >= 16:
                    self.motors = list(struct.unpack("<4f", data[:16]))
                    if self.trace is not None:
                        self.trace.append((time.monotonic(), self.motors[0]))
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
# Low-drag plant: the nav velocity loop (ap_velocity_*) enforces cruise speed,
# so the drag no longer needs to mask integrator overshoot. base_config sets
# ap_velocity_drag_coeff to match this plant (atan(K_DRAG*v/g) over 5-7.5 m/s).
K_DRAG = 0.8               # 1/s, linear drag: 22 deg of tilt holds ~5 m/s
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

    def body_rates(self):
        return self.rates

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


# --- fixed wing -------------------------------------------------------------
# A 1 m flying wing of about 1 kg: wing loading 5 kg/m^2, aspect ratio 4. Forces are per unit mass. On the FC's
# default throttle schedule it flies level at about 16.3 m/s on 53% motor at a lift to drag of 6.3, and glides at
# up to 8:1 at 11-12 m/s with the motor off; it stalls at 8.8 m/s.

WING_KL = 0.12             # lift acceleration per (m/s)^2 per unit CL: rho * S / (2 m)
WING_CL_ALPHA = 5.0        # per radian
WING_ALPHA_STALL = math.radians(12.0)
WING_CL_POST_STALL = 0.4   # fraction of CL_max retained once stalled
WING_STALL_BREAK = math.radians(6.0)   # past the stall the lift falls from CL_max to the post-stall level over this
WING_CD0 = 0.0047          # parasitic drag acceleration per (m/s)^2: CD0 0.039
WING_CDI = 0.012           # induced drag acceleration per (m/s)^2 per CL^2: span efficiency 0.8
WING_INCIDENCE = math.radians(3.0)   # the zero-lift line above the FC's level
WING_THRUST_MAX = 9.0      # m/s^2 of static thrust at full throttle, growing with the throttle squared
WING_PROP_SPEED = 40.0     # m/s of airspeed at which the propeller stops pulling
WING_SIDE_FORCE = 0.036    # side acceleration per (m/s)^2 per radian of sideslip
WING_WEATHERVANE_TAU = 0.3   # s, the fins turning the nose into the relative wind
WING_V_STALL = math.sqrt(GRAVITY / (WING_KL * WING_CL_ALPHA * WING_ALPHA_STALL))
WING_V_REF = 15.0          # m/s, the airspeed at which the surfaces have full authority
WING_ROLL_GAIN = 5.0       # rad/s of roll rate per unit elevon at V_REF
WING_PITCH_GAIN = 3.0      # rad/s of pitch rate per unit elevon at V_REF
WING_RATE_TAU = 0.10
WING_SERVO_SPAN = 500.0    # PWM counts from centre to full deflection
WING_FRICTION = 0.4        # sliding friction coefficient on the ground
WING_RESTITUTION = 0.15    # of the sink at a contact harder than WING_BOUNCE_SINK, bounced back
WING_BOUNCE_SINK = 1.0     # m/s
WING_SETTLE_TAU = 0.2      # s, the ground rolls the wings level
WING_GROUND_PITCH = (math.radians(-14.0), math.radians(6.0))   # nose-down positive: tail and nose on the ground
WING_SLIDE_NOISE = 0.5     # m/s^2 of vibration at 5 m/s over the ground, growing with the speed
# a contact past any of these is a crash
WING_CRASH_SINK = 2.5      # m/s
WING_CRASH_ROLL = math.radians(20.0)
WING_CRASH_NOSE_DOWN = math.radians(15.0)
WING_CRASH_SPEED = 25.0    # m/s over the ground


def wrap_pi(rad):
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


class WingMotionModel:
    """Fixed-wing point-mass plant: elevons -> attitude, attitude -> lift/drag.

    Honours the same contract as MotionModel (ENU pos/vel/accel, roll right
    positive, pitch nose-DOWN positive, yaw compass CW positive) so FdmFeed's
    existing frame handling carries over unchanged.

    Control authority scales with airspeed, so an airframe sitting still in the
    thrower's hand cannot move its own surfaces.

    vel is the ground velocity (what GPS reports), integrated from the forces, so it never steps. The aerodynamics see
    the air-relative velocity, vel - wind, with wind a constant ENU vector: lift square to it, drag against it, a side
    force turning it towards the nose, and the fins turning the nose towards it.
    """

    def __init__(self, wind=(0.0, 0.0, 0.0), ground=None):
        self.wind = list(wind)
        self.ground = ground or (lambda e, n: 0.0)    # the ground's height at east, north metres from home
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.accel = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.rates = [0.0, 0.0, 0.0]
        self.held = True          # in the thrower's hand until launched
        self.crashed = False
        self.crash_reason = None
        self.grounded = False     # sliding on the ground after a contact
        # the first ground contact: dict(t, e, n, ve, vn, vz, gs, airspeed, roll, pitch, stalled)
        self.contact = None
        self.stalled = False
        self.stall_s = 0.0        # how long the current airborne stall has lasted
        self.longest_stall_s = 0.0
        self.alpha = 0.0
        self.t = 0.0
        self._noise = random.Random(1)
        self._throw_ticks = 0
        self._throw_accel = 0.0

    # -- state the scenarios assert on -------------------------------------

    @property
    def airspeed(self):
        return math.sqrt(sum((v - w) ** 2 for v, w in zip(self.vel, self.wind)))

    def ground_at(self, e=None, n=None):
        return self.ground(self.pos[0] if e is None else e, self.pos[1] if n is None else n)

    def ground_rise(self, dt):
        """The rate at which the ground under the aircraft rises as it moves over it."""
        e, n = self.pos[0], self.pos[1]
        return (self.ground_at(e + self.vel[0] * dt, n + self.vel[1] * dt) - self.ground_at(e, n)) / dt

    def on_ground(self):
        return self.pos[2] <= self.ground_at() + 0.001

    def body_rates(self):
        """Gyro rates from the Euler rates the model integrates, in the model's
        conventions (roll right+, nose-down+, yaw CW+). A banked turn reads on
        both pitch and yaw, as on a real airframe."""
        roll_rate, pitch_rate, yaw_rate = self.rates
        sin_r, cos_r = math.sin(self.roll), math.cos(self.roll)
        sin_p, cos_p = math.sin(self.pitch), math.cos(self.pitch)
        return [
            roll_rate + yaw_rate * sin_p,
            pitch_rate * cos_r - yaw_rate * sin_r * cos_p,
            pitch_rate * sin_r + yaw_rate * cos_r * cos_p,
        ]

    # -- launch injection ---------------------------------------------------

    def hold_in_hand(self, pitch_deg=10.0, altitude_m=0.0):
        """Pin the airframe as if carried: still, nose slightly up, 1 g only. By default it sits on
        the ground, where it is armed: a landing at home takes the arming point for the ground."""
        self.held = True
        self.vel = [0.0, 0.0, 0.0]
        self.accel = [0.0, 0.0, 0.0]
        self.rates = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = math.radians(-pitch_deg)   # nose-down positive
        self.pos[2] = altitude_m

    def hand_launch(self, speed_ms=8.0, duration_s=0.25):
        """Throw along the nose: the hand carries the weight and accelerates the airframe to speed_ms, which the FC
        sees as body-axis specific force for the whole duration."""
        self._throw_accel = speed_ms / duration_s
        self._throw_ticks = max(1, int(round(duration_s / 0.02)))
        self.held = False

    # -- plant --------------------------------------------------------------

    def _elevons(self, servos):
        """Demix two elevon servos into normalised pitch and roll commands."""
        if not servos or len(servos) < 2:
            return 0.0, 0.0
        left = (servos[0] - 1500.0) / WING_SERVO_SPAN
        right = (servos[1] - 1500.0) / WING_SERVO_SPAN
        pitch_cmd = (left + right) / 2.0
        roll_cmd = (left - right) / 2.0
        return max(-1.0, min(1.0, pitch_cmd)), max(-1.0, min(1.0, roll_cmd))

    @staticmethod
    def _thrust(throttle, airspeed):
        return WING_THRUST_MAX * throttle * throttle * max(0.0, 1.0 - airspeed / WING_PROP_SPEED)

    def _lift_coefficient(self):
        self.stalled = abs(self.alpha) > WING_ALPHA_STALL
        if not self.stalled:
            return WING_CL_ALPHA * self.alpha
        past = min(1.0, (abs(self.alpha) - WING_ALPHA_STALL) / WING_STALL_BREAK)
        return math.copysign(WING_CL_ALPHA * WING_ALPHA_STALL * (1.0 - (1.0 - WING_CL_POST_STALL) * past), self.alpha)

    def take_stall_record(self):
        """The longest airborne stall so far, starting the record afresh."""
        longest, self.longest_stall_s = self.longest_stall_s, 0.0
        return longest

    def _attitude(self, dt, airspeed, pitch_cmd, roll_cmd):
        """Elevon authority scales with airspeed, zero at a standstill."""
        authority = min(1.0, airspeed / WING_V_REF)
        target = (WING_ROLL_GAIN * roll_cmd * authority, WING_PITCH_GAIN * pitch_cmd * authority)
        for i in range(2):
            self.rates[i] += (target[i] - self.rates[i]) * min(1.0, dt / WING_RATE_TAU)
        self.roll = max(-1.2, min(1.2, self.roll + self.rates[0] * dt))
        self.pitch = max(-1.2, min(1.2, self.pitch + self.rates[1] * dt))

    def _integrate(self, dt, acc):
        self.accel = list(acc)
        for i in range(3):
            self.vel[i] += acc[i] * dt
            self.pos[i] += self.vel[i] * dt

    def _touch_down(self, dt):
        """Ground contact from the air: record the first, crash past the limits, bounce a hard one
        back up, and otherwise settle onto the ground."""
        gs = math.hypot(self.vel[0], self.vel[1])
        rise = self.ground_rise(dt) if dt > 0 else 0.0
        vz = self.vel[2] - rise
        if self.contact is None:
            self.contact = dict(t=self.t, e=self.pos[0], n=self.pos[1], ve=self.vel[0], vn=self.vel[1], vz=vz, gs=gs,
                                airspeed=self.airspeed, roll=math.degrees(self.roll), pitch=-math.degrees(self.pitch),
                                stalled=self.stalled)
        reasons = [why for why, hit in (
            (f"sink {-vz:.1f} m/s", vz < -WING_CRASH_SINK),
            (f"wing strike at {math.degrees(self.roll):+.0f} deg of bank", abs(self.roll) > WING_CRASH_ROLL),
            (f"nose in at {math.degrees(self.pitch):.0f} deg nose down", self.pitch > WING_CRASH_NOSE_DOWN),
            (f"{gs:.1f} m/s over the ground", gs > WING_CRASH_SPEED),
        ) if hit]
        self.pos[2] = self.ground_at()
        if reasons:
            self.crashed = True
            self.crash_reason = ", ".join(reasons)
            self.vel = [0.0, 0.0, 0.0]
            self.accel = [0.0, 0.0, 0.0]
            self.rates = [0.0, 0.0, 0.0]
        else:
            if -vz > WING_BOUNCE_SINK:
                self.vel[2] = rise - WING_RESTITUTION * vz
            else:
                self.vel[2] = rise
                self.grounded = True
            if dt > 0:
                self.accel[2] += (self.vel[2] - rise - vz) / dt

    def _slide(self, dt, throttle, pitch_cmd):
        """On the ground: thrust along the heading, drag against the relative wind and friction against the
        motion, the wings settling level and the nose held between its tail and its nose; airborne again once
        the lift carries it."""
        sin_y, cos_y = math.sin(self.yaw), math.cos(self.yaw)
        air = (self.vel[0] - self.wind[0], self.vel[1] - self.wind[1])
        air_along = air[0] * sin_y + air[1] * cos_y
        gs = math.hypot(self.vel[0], self.vel[1])

        self.rates[0] = -self.roll / WING_SETTLE_TAU
        self.roll += self.rates[0] * dt
        authority = min(1.0, abs(air_along) / WING_V_REF)
        self.rates[1] += (WING_PITCH_GAIN * pitch_cmd * authority - self.rates[1]) * min(1.0, dt / WING_RATE_TAU)
        if gs < 1.0:
            self.rates = [0.0, 0.0, 0.0]
        self.rates[2] = 0.0
        self.pitch = max(WING_GROUND_PITCH[0], min(WING_GROUND_PITCH[1], self.pitch + self.rates[1] * dt))

        self.alpha = -self.pitch + WING_INCIDENCE
        self.stalled = False
        cl = WING_CL_ALPHA * max(-WING_ALPHA_STALL, min(WING_ALPHA_STALL, self.alpha))
        lift = WING_KL * air_along * air_along * cl
        drag = (WING_CD0 + WING_CDI * cl * cl) * math.hypot(*air)
        thrust = self._thrust(throttle, max(0.0, air_along))
        e, n, h = self.pos[0], self.pos[1], 0.5
        slope = ((self.ground_at(e + h, n) - self.ground_at(e - h, n)) / (2.0 * h),
                 (self.ground_at(e, n + h) - self.ground_at(e, n - h)) / (2.0 * h))
        push = [thrust * sin_y - drag * air[0] - GRAVITY * slope[0],
                thrust * cos_y - drag * air[1] - GRAVITY * slope[1]]
        friction = WING_FRICTION * max(0.0, GRAVITY - lift * math.cos(self.roll))
        if gs > 1e-3:
            new_h = [v + (p - friction * v / gs) * dt for v, p in zip(self.vel, push)]
            if new_h[0] * self.vel[0] + new_h[1] * self.vel[1] < 0.0:
                new_h = [0.0, 0.0]
        elif math.hypot(*push) > friction:
            scale = 1.0 - friction / math.hypot(*push)
            new_h = [p * scale * dt for p in push]
        else:
            new_h = [0.0, 0.0]

        ground = self.ground_at()
        new_pos = (self.pos[0] + new_h[0] * dt, self.pos[1] + new_h[1] * dt)
        new_ground = self.ground_at(*new_pos)
        new_vel = [new_h[0], new_h[1], (new_ground - ground) / dt]
        if lift * math.cos(self.roll) > GRAVITY:
            self.grounded = False
        sigma = WING_SLIDE_NOISE * math.hypot(*new_h) / 5.0
        for i in range(3):
            self.accel[i] = (new_vel[i] - self.vel[i]) / dt + (self._noise.gauss(0.0, sigma) if sigma > 0.0 else 0.0)
        self.vel = new_vel
        self.pos[0], self.pos[1] = new_pos
        self.pos[2] = new_ground

    def _throw(self, dt, pitch_cmd, roll_cmd):
        self._attitude(dt, self.airspeed, pitch_cmd, roll_cmd)
        nose_up = -self.pitch
        self._integrate(dt, (self._throw_accel * math.sin(self.yaw) * math.cos(nose_up),
                             self._throw_accel * math.cos(self.yaw) * math.cos(nose_up),
                             self._throw_accel * math.sin(nose_up)))
        self._throw_ticks -= 1

    def _fly(self, dt, throttle, pitch_cmd, roll_cmd):
        air = [v - w for v, w in zip(self.vel, self.wind)]
        v = math.sqrt(sum(a * a for a in air))
        self._attitude(dt, v, pitch_cmd, roll_cmd)

        nose_up = -self.pitch
        sin_y, cos_y = math.sin(self.yaw), math.cos(self.yaw)
        nose = (sin_y * math.cos(nose_up), cos_y * math.cos(nose_up), math.sin(nose_up))
        u = [a / v for a in air] if v > 0.5 else list(nose)
        horiz = math.hypot(u[0], u[1])
        if horiz < 1e-3:
            u = list(nose)
            horiz = math.hypot(u[0], u[1])
        gamma = math.asin(max(-1.0, min(1.0, u[2])))
        course = math.atan2(u[0], u[1])
        up = (-u[0] * u[2] / horiz, -u[1] * u[2] / horiz, horiz)    # square to the air path, upwards
        right = (u[1] / horiz, -u[0] / horiz, 0.0)                  # square to the air path, level, to the right

        self.alpha = nose_up + WING_INCIDENCE - gamma
        cl = self._lift_coefficient()
        lift = WING_KL * v * v * cl
        drag = (WING_CD0 + WING_CDI * cl * cl) * v * v
        side = WING_SIDE_FORCE * v * v * math.sin(wrap_pi(self.yaw - course))
        thrust = self._thrust(throttle, v)
        sin_r, cos_r = math.sin(self.roll), math.cos(self.roll)
        acc = [thrust * nose[i] - drag * u[i] + lift * (up[i] * cos_r + right[i] * sin_r) + side * right[i]
               for i in range(3)]
        acc[2] -= GRAVITY

        # bank-to-turn: no rudder, the air path turns with the lift and the nose follows it
        turn_rate = (lift * sin_r + side) / max(v * horiz, 1.0)
        self.rates[2] = turn_rate + wrap_pi(course - self.yaw) / WING_WEATHERVANE_TAU
        self.yaw += self.rates[2] * dt
        self._integrate(dt, acc)
        if self.pos[2] < self.ground_at():
            self._touch_down(dt)

    def step(self, dt, m, servos=None):
        self.t += dt
        if self.crashed:
            self.accel = [0.0, 0.0, 0.0]
            return

        throttle = m[0] if m else 0.0
        pitch_cmd, roll_cmd = self._elevons(servos)
        if self.grounded:
            self._slide(dt, throttle, pitch_cmd)
        elif self.held and self._throw_ticks == 0:
            # carried: gravity only, nothing else moves
            self.vel = [0.0, 0.0, 0.0]
            self.accel = [0.0, 0.0, 0.0]
            self.rates = [0.0, 0.0, 0.0]
        elif self._throw_ticks > 0:
            self._throw(dt, pitch_cmd, roll_cmd)
        else:
            self._fly(dt, throttle, pitch_cmd, roll_cmd)

        if self.stalled and not self.grounded and not self.held:
            self.stall_s += dt
            self.longest_stall_s = max(self.longest_stall_s, self.stall_s)
        else:
            self.stall_s = 0.0


class PwmRawFeed(threading.Thread):
    """Listens for SITL's raw PWM outputs (servo_packet_raw on UDP 9001).

    The C struct is uint16_t motorCount followed by float[16], so the floats
    start at offset 4 after padding: 68 bytes in total.
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", PWM_RAW_PORT))
        self.sock.settimeout(0.2)
        self.motor_count = 0
        self.channels = [1500.0] * 16
        self.running = True

    @property
    def servos(self):
        return self.channels[self.motor_count:]

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(128)
                if len(data) >= 68:
                    unpacked = struct.unpack("<H2x16f", data[:68])
                    self.motor_count = unpacked[0]
                    self.channels = list(unpacked[1:])
            except socket.timeout:
                pass
            except OSError:
                break

    def shutdown(self):
        self.running = False
        if self.is_alive():
            self.join(timeout=1.0)
        self.sock.close()


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


class SensorErrors:
    """Seeded sensor errors: white noise on the gyro, the accelerometer and the altitude (one altitude feeds both
    the baro and the GPS), and a GPS that reports GPS_DELAY_S late with velocity noise and a position error that
    wanders with a GPS_POS_TAU_S correlation time."""

    GYRO_DPS = 0.5
    ACC_MSS = 0.3
    ALT_M = 0.15
    GPS_POS_M = 0.5
    GPS_POS_TAU_S = 20.0
    GPS_VEL_MS = 0.1
    GPS_DELAY_S = 0.1

    def __init__(self, seed=7):
        self.rng = random.Random(seed)
        self.pos_err = [0.0, 0.0]
        self.queue = collections.deque()

    def gyro(self, rates):
        return [r + math.radians(self.rng.gauss(0.0, self.GYRO_DPS)) for r in rates]

    def acc(self, f):
        return [a + self.rng.gauss(0.0, self.ACC_MSS) for a in f]

    def altitude(self, alt):
        return alt + self.rng.gauss(0.0, self.ALT_M)

    def gps(self, t, dt, east, north, vel):
        """The (east, north, velocity) the GPS reports at t for the true values."""
        k = min(1.0, dt / self.GPS_POS_TAU_S)
        for i in range(2):
            self.pos_err[i] += -k * self.pos_err[i] + self.GPS_POS_M * math.sqrt(2.0 * k) * self.rng.gauss(0.0, 1.0)
        self.queue.append((t, east + self.pos_err[0], north + self.pos_err[1],
                           [v + self.rng.gauss(0.0, self.GPS_VEL_MS) for v in vel]))
        while len(self.queue) > 1 and self.queue[1][0] <= t - self.GPS_DELAY_S:
            self.queue.popleft()
        return self.queue[0][1:]


class FdmFeed(threading.Thread):
    """50 Hz fdm_packet stream driven by the motion model.

    Emits in the Gazebo-bridge conventions the default SITL build expects:
    quaternion pre-multiplied by Rz(-90deg) (the FC re-applies Rz(+90deg)),
    gyro in the plugin sensor frame (pitch and yaw negated from the model's
    nose-down/compass-CW conventions), and GPS lat/lon mirrored around the
    first packet's origin (the FC un-mirrors).
    """

    def __init__(self, motors=None, initial_yaw_deg=0.0, status=None, model=None, pwm_raw=None, sensors=None):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.model = model if model is not None else MotionModel()
        self.pwm_raw = pwm_raw
        self.sensors = sensors    # SensorErrors, or None for exact sensors
        self.model.yaw = math.radians(initial_yaw_deg)
        self.motors = motors
        self.status = status
        self.sid = uuid.uuid4().hex[:8]  # telemetry session id: lets visualisers detect restarts
        self.running = True
        self.gps_valid = True     # False emits out-of-range lat/lon: the FC's GPS goes dark
        self.altitude_bias_m = 0.0   # added to the altitude the FC reads, baro and GPS alike
        self.history = []         # (t, east, north, up, ve, vn, vu, heading_deg) at ~10 Hz
        self._hist_lock = threading.Lock()
        self._hist_decim = 0
        self.t0 = time.monotonic()

    def move_east(self, metres):
        self.model.pos[0] += metres

    def distance_from_home(self):
        return math.hypot(self.model.pos[0], self.model.pos[1])

    def distance_to_wp(self, east_m, north_m):
        return math.hypot(self.model.pos[0] - east_m, self.model.pos[1] - north_m)

    def ground_speed(self):
        return math.hypot(self.model.vel[0], self.model.vel[1])

    def heading_deg(self):
        return math.degrees(self.model.yaw) % 360.0

    def snapshot_history(self):
        with self._hist_lock:
            return list(self.history)

    def max_altitude(self):
        return max((s[3] for s in self.snapshot_history()), default=0.0)

    def time_to_home(self, radius_m=10.0, after_t=0.0):
        """First recorded time the craft is within radius_m of home, after after_t."""
        for s in self.snapshot_history():
            if s[0] > after_t and math.hypot(s[1], s[2]) < radius_m:
                return s[0]
        return None

    def touchdown(self, after_t=0.0):
        """(t, east, north) of the first on-ground sample following airborne flight."""
        airborne = False
        for s in self.snapshot_history():
            if s[0] < after_t:
                continue
            if s[3] > 1.0:
                airborne = True
            elif airborne and s[3] <= 0.01:
                return (s[0], s[1], s[2])
        return None

    def max_distance_from_home(self, after_t=0.0):
        return max((math.hypot(s[1], s[2]) for s in self.snapshot_history() if s[0] >= after_t), default=0.0)

    def now_t(self):
        return time.monotonic() - self.t0

    def run(self):
        last = time.monotonic()
        while self.running:
            now = time.monotonic()
            dt = min(0.1, now - last)
            last = now
            m = self.motors.motors if self.motors else [0.0] * 4
            if self.pwm_raw is not None:
                self.model.step(dt, m, self.pwm_raw.servos)
            else:
                self.model.step(dt, m)

            self._hist_decim += 1
            if self._hist_decim >= 5:  # ~10 Hz of the 50 Hz loop
                self._hist_decim = 0
                with self._hist_lock:
                    self.history.append((now - self.t0,
                                         self.model.pos[0], self.model.pos[1], self.model.pos[2],
                                         self.model.vel[0], self.model.vel[1], self.model.vel[2],
                                         self.heading_deg(), math.degrees(self.model.pitch)))

            lat_true = HOME_LAT + self.model.pos[1] / M_PER_DEG
            lon_true = HOME_LON + self.model.pos[0] / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))
            altitude = HOME_ALT_M + self.model.pos[2] + self.altitude_bias_m
            lat_gps, lon_gps, vel_gps = lat_true, lon_true, self.model.vel

            if TELEMETRY_PORT:
                try:
                    # Ground-truth state for external visualisers. The model
                    # keeps pitch nose-down/yaw-CW positive; emit display
                    # conventions (pitch nose-up positive) once, here.
                    self.sock.sendto(json.dumps({
                        "sid": self.sid,
                        "t": now - self.t0,
                        "pos": list(self.model.pos),
                        "vel": list(self.model.vel),
                        "att": [math.degrees(self.model.roll),
                                -math.degrees(self.model.pitch),
                                math.degrees(self.model.yaw) % 360.0],
                        "rates": [math.degrees(self.model.rates[0]),
                                  -math.degrees(self.model.rates[1]),
                                  math.degrees(self.model.rates[2])],
                        "motors": list(m),
                        "lat": lat_true,
                        "lon": lon_true,
                        "alt": HOME_ALT_M + self.model.pos[2],
                        "gps": bool(self.gps_valid),
                        "armed": self.status.armed if self.status else None,
                        "modes": self.status.modes if self.status else [],
                        "home": [HOME_LAT, HOME_LON, HOME_ALT_M],
                    }).encode(), ("127.0.0.1", TELEMETRY_PORT))
                except OSError:
                    pass  # fire-and-forget; a visualiser must never affect a scenario

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
            gyro = self.model.body_rates()
            if self.sensors is not None:
                f_body = self.sensors.acc(f_body)
                gyro = self.sensors.gyro(gyro)
                altitude = self.sensors.altitude(altitude)
                east, north, vel_gps = self.sensors.gps(now - self.t0, dt, self.model.pos[0], self.model.pos[1],
                                                        self.model.vel)
                lat_gps = HOME_LAT + north / M_PER_DEG
                lon_gps = HOME_LON + east / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))

            # Out-of-range lat/lon = GPS-loss sentinel; the FC skips the
            # virtual GPS update and its receive timeout trips, while IMU
            # feeds stay live. (NaN would be folded away by -ffast-math.)
            lon_pkt = 2.0 * HOME_LON - lon_gps if self.gps_valid else 999.0
            lat_pkt = 2.0 * HOME_LAT - lat_gps if self.gps_valid else 999.0
            pkt = struct.pack(
                "<18d",
                now - self.t0,
                # Gazebo-plugin gyro frame: roll right +, pitch nose-up +,
                # yaw CCW +. The model keeps nose-down/CW positive (compass
                # conventions), so pitch and yaw are negated on emit.
                gyro[0], -gyro[1], -gyro[2],
                -f_body[0], -f_body[1], -f_body[2],                              # negated NWU-body specific force
                q[0], q[1], q[2], q[3],
                vel_gps[0], vel_gps[1], vel_gps[2],                              # ENU m/s
                lon_pkt,                                                         # mirrored for the bridge
                lat_pkt,
                altitude,
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
        # serialises request/reply pairs: the status poller thread shares this
        # connection with scenario bodies
        self.lock = threading.Lock()

    def request(self, cmd, payload=b"", timeout=2.0):
        with self.lock:
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
        extra_count = p[15]
        off = 16 + extra_count
        arming_count = p[off]
        arming_flags = struct.unpack_from("<I", p, off + 1)[0]
        # The first 32 bits sit at offset 6; any box index past 31 is carried in
        # the extra bytes at offset 16. A wing with GPS has more than 32 active
        # boxes, so dropping those makes the high boxes invisible.
        mode_bits = int.from_bytes(bytes(p[6:10]) + bytes(p[16:16 + extra_count]), "little")
        active = {self.boxids[i] for i in range(len(self.boxids)) if mode_bits & (1 << i)}
        return {"modes": active, "arming_flags": arming_flags, "arming_count": arming_count}

    def modes(self):
        return self.status()["modes"]

    def attitude(self):
        """Estimated (roll, pitch) in degrees: roll right +, pitch nose-down +."""
        roll, pitch = struct.unpack_from("<hh", self.msp.request(MSP_ATTITUDE))
        return roll / 10.0, pitch / 10.0

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

    def yaw_deg(self):
        p = self.msp.request(MSP_ATTITUDE)
        return struct.unpack_from("<h", p, 4)[0] % 360

    def debug_values(self):
        p = self.msp.request(MSP_DEBUG)
        return list(struct.unpack_from(f"<{len(p) // 2}h", p))

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


class StatusPoller(threading.Thread):
    """5 Hz MSP_STATUS poll feeding true arm/mode state into the telemetry
    fan-out, and keeping the MSP link alive: SITL drops a TCP client idle for
    120 s. Read-only observer: errors are swallowed and the last state kept,
    so it can never fail a scenario."""

    BOX_NAMES = {
        BOX_ARM: "ARM",
        BOX_ALTHOLD: "ALTHOLD",
        BOX_POSHOLD: "POSHOLD",
        BOX_FAILSAFE: "FAILSAFE",
        BOX_GPSRESCUE: "GPSRESCUE",
        BOX_AUTOPILOT: "AUTOPILOT",
        BOX_LAUNCH: "LAUNCH",
    }

    def __init__(self, sitl):
        super().__init__(daemon=True)
        self.sitl = sitl
        self.running = True
        self.armed = False
        self.modes = []

    def run(self):
        while self.running:
            try:
                if self.sitl.msp is not None:
                    modes = self.sitl.modes()
                    self.armed = BOX_ARM in modes
                    self.modes = [self.BOX_NAMES.get(b, f"BOX{b}") for b in sorted(modes) if b != BOX_ARM]
            except (TimeoutError, RuntimeError, OSError):
                pass
            time.sleep(0.2)

    def shutdown(self):
        self.running = False


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
WP_NORTH90_LAT = HOME_LAT + 90.0 / M_PER_DEG  # far leg for the backwards-engage mission
WP_EAST90_LON = HOME_LON + 90.0 / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))  # corner for the face-the-next-waypoint mission
# ~130 deg corner: a 60 m north leg into wp0, then out to (42 m east, 25 m north),
# so the outgoing leg bears ~130 deg and the pre-turn swings the nose past 90 deg
# off the inbound leg
WP_NORTH60_LAT = HOME_LAT + 60.0 / M_PER_DEG
WP_CORNER_LAT = HOME_LAT + 25.0 / M_PER_DEG
WP_CORNER_LON = HOME_LON + 42.0 / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))


def base_config(extra):
    return [
        "feature GPS",
        # the executor's state, abort reason and leg: a scenario that fails leaves a log that says
        # what nav was doing when it did
        "set debug_mode = FLIGHT_PLAN",
        "set gps_provider = VIRTUAL",
        "set failsafe_procedure = AUTO-LAND",
        "set failsafe_delay = 10",
        "set small_angle = 180",
        "aux 0 0 0 1700 2100 0 0",   # ARM on AUX1
        "aux 1 56 1 1700 2100 0 0",  # AUTOPILOT on AUX2
        "aux 2 1 2 1700 2100 0 0",   # ANGLE on AUX3 (heading-validation flight)
        # compassEnabledAndCalibrated() requires stored calibration values: the
        # virtual compass is never calibrated, so seed a negligible bias to mark
        # it calibrated, or the heading is never trusted and nav stands down
        "set mag_calibration = 1,1,1",
        # Unified velocity-primitive controller: cruise tilt is carried by the
        # virtual-distance integral, so drag compensation is a small term kept
        # well below the D (velocity) gain rather than the cruise feedforward.
        "set ap_velocity_drag_coeff = 50",
        # 10 m above home, 5 m/s — low and quick keeps landing scenarios short
        f"waypoint insert 0 {WP_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 none",
    ] + extra


def boot_and_arm(sitl, rc, fdm):
    """Boot, GPS fix, accelerometer recalibration, arm in ANGLE."""
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


def boot_and_engage(sitl, rc, fdm):
    """Common preamble: boot, GPS fix, arm, raise throttle, engage AUTOPILOT."""
    boot_and_arm(sitl, rc, fdm)

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
    # Mid-leg cruise: sample in the plateau (past the accel ramp, before the
    # ~42 m braking taper) and check the velocity loop holds the commanded
    # 5 m/s without overshoot.
    cruise_samples = []

    def reached_or_sample():
        d = fdm.distance_to_wp(0.0, 300.0)
        if fdm.distance_from_home() > 50.0 and d > 100.0:
            cruise_samples.append(math.hypot(fdm.model.vel[0], fdm.model.vel[1]))
        return d < 8.0

    wait_for(
        "waypoint reached (within 8 m, ground truth)",
        lambda: reached_or_sample(),
        timeout=150,
        interval=1.0,
    )
    assert len(cruise_samples) >= 5, f"cruise plateau too short: {len(cruise_samples)} samples"
    cruise_avg = sum(cruise_samples) / len(cruise_samples)
    cruise_max = max(cruise_samples)
    assert 0.8 * 5.0 <= cruise_avg <= 1.2 * 5.0, f"cruise speed off target: avg {cruise_avg:.2f} m/s"
    assert cruise_max <= 1.3 * 5.0, f"cruise overshoot: peak {cruise_max:.2f} m/s"
    log(f"cruise avg {cruise_avg:.2f} m/s, peak {cruise_max:.2f} m/s over {len(cruise_samples)} samples")
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


def scenario_mission_engage_backwards(sitl, rc, fdm):
    """Engage with the nose pointing away from the first leg (initial_yaw_deg=180,
    leg runs north). In the default VELOCITY yaw mode no course develops while the
    craft sits still, so the executor must rotate the nose onto the leg and fly it
    rather than freezing the carrot into a STALL abort."""
    boot_and_engage(sitl, rc, fdm)

    # Departing at all proves it didn't deadlock: a frozen carrot never moves,
    # develops no course, and aborts STALLED after 30 s.
    wait_for(
        "rotates onto the leg and departs (>15 m from home)",
        lambda: fdm.distance_from_home() > 15.0,
        timeout=40,
    )

    def on_leg():
        err = (fdm.heading_deg() - 0.0 + 180.0) % 360.0 - 180.0
        return abs(err) < 30.0

    wait_for("nose swung onto the northbound leg (~000)", on_leg, timeout=25)

    wait_for(
        "reaches the far waypoint (ground truth)",
        lambda: fdm.distance_to_wp(0.0, 90.0) < 10.0,
        timeout=120,
        interval=1.0,
    )
    assert BOX_ARM in sitl.modes(), "unexpected disarm on the backwards-engage mission"
    log(f"rotated onto the leg from a backwards engage, heading {fdm.heading_deg():.0f} deg")


def scenario_mission_corner(sitl, rc, fdm):
    """A ~130 deg corner. The pre-turn deliberately swings the nose past 90 deg
    off the inbound leg approaching the gate; the march gate must exempt the
    pre-turn so the craft carries corner speed through the gate instead of
    freezing in it (the hairpin-stall bug)."""
    boot_and_engage(sitl, rc, fdm)

    wait_for(
        "flies the first leg to the corner waypoint",
        lambda: fdm.distance_to_wp(0.0, 60.0) < 12.0,
        timeout=90,
        interval=1.0,
    )

    # Sample ground speed while crossing the corner: a stalled gate would drop it
    # toward zero; a working pre-turn carries it through near the corner speed.
    corner_samples = []

    def carried_through():
        if fdm.distance_to_wp(0.0, 60.0) < 20.0:
            corner_samples.append(math.hypot(fdm.model.vel[0], fdm.model.vel[1]))
        return fdm.distance_to_wp(42.0, 25.0) < 10.0

    wait_for(
        "carries through the corner to the second waypoint",
        carried_through,
        timeout=120,
        interval=1.0,
    )
    assert corner_samples, "never sampled near the corner"
    corner_min = min(corner_samples)
    assert corner_min > 0.8, f"stalled in the corner: min ground speed {corner_min:.2f} m/s"
    assert BOX_ARM in sitl.modes(), "unexpected disarm during the corner mission"
    log(f"carried the corner, min ground speed {corner_min:.2f} m/s over {len(corner_samples)} samples")


def scenario_mission_land(sitl, rc, fdm):
    """LAND waypoint: fly the first leg north, divert to the offset LAND
    waypoint, arrive through the 3D hold gate, loiter for the waypoint
    duration, descend, and disarm on touchdown (impact jerk; the estimator's
    vz is unreliable when grounded)."""
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
    # 5 s pre-descent loiter: shortly after arrival the vehicle must still be
    # holding altitude (an immediate 2 m/s descent would be ~4 m down by now)
    time.sleep(2.0)
    assert BOX_ARM in sitl.modes(), "disarmed during the loiter"
    assert fdm.model.pos[2] > 7.0, f"descended during the loiter: alt {fdm.model.pos[2]:.1f} m"
    log(f"loitering at {fdm.model.pos[2]:.1f} m before descent")
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


def scenario_mission_takeoff(sitl, rc, fdm):
    """TAKEOFF waypoint: climb in place to the waypoint altitude (its lat/lon
    are advisory), then fly the following leg. The climb must not translate."""
    boot_and_engage(sitl, rc, fdm)
    t_engage = fdm.now_t()

    wait_for(
        "climb through 12 m (TAKEOFF target 15 m)",
        lambda: fdm.model.pos[2] > 12.0,
        timeout=90,
        interval=0.5,
    )
    t_top = fdm.now_t()

    # Horizontal drift during the climb, relative to where the mission engaged
    # (TAKEOFF holds the current position, not home).
    climb = [s for s in fdm.snapshot_history() if t_engage <= s[0] <= t_top]
    assert climb, "no recorded samples during the climb"
    e0, n0 = climb[0][1], climb[0][2]
    drift = max(math.hypot(s[1] - e0, s[2] - n0) for s in climb)
    assert drift < 8.0, f"translated {drift:.1f} m during the TAKEOFF climb"
    log(f"climbed to {fdm.model.pos[2]:.1f} m with {drift:.1f} m drift")

    wait_for(
        "leg to the north waypoint after the climb",
        lambda: fdm.distance_to_wp(0.0, 40.0) < 10.0,
        timeout=90,
        interval=1.0,
    )
    assert BOX_ARM in sitl.modes(), "unexpected disarm during the takeoff mission"


def hold_window_samples(fdm, centre_e, centre_n, t0, t1):
    """(distances, azimuth sweep in rad) of history samples in [t0, t1],
    measured about the hold point. Sweep accumulates wrapped step deltas, so
    systematic circulation grows it while hover noise cancels out."""
    pts = [s for s in fdm.snapshot_history() if t0 <= s[0] <= t1]
    dists = [math.hypot(s[1] - centre_e, s[2] - centre_n) for s in pts]
    azimuths = [math.atan2(s[2] - centre_n, s[1] - centre_e) for s in pts]
    sweep = 0.0
    for a, b in zip(azimuths, azimuths[1:]):
        sweep += (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return dists, sweep


def scenario_mission_orbit(sitl, rc, fdm):
    """HOLD with the ORBIT pattern: after arriving at the hold point the
    vehicle must circulate around it on the hold radius for the duration."""
    boot_and_engage(sitl, rc, fdm)

    wait_for(
        "arrival at the hold point",
        lambda: fdm.distance_to_wp(0.0, 40.0) < 10.0,
        timeout=90,
        interval=1.0,
    )
    t_arrive = fdm.now_t()

    # Analysis window: skip 12 s (arrival braking + pattern spin-up), observe
    # 40 s of the 60 s hold. Carrot rate 0.25 rad/s -> ~1.6 laps in the window.
    wait_for("orbit window elapsed", lambda: fdm.now_t() > t_arrive + 52.0,
             timeout=70, interval=2.0)
    dists, sweep = hold_window_samples(fdm, 0.0, 40.0, t_arrive + 12.0, t_arrive + 52.0)
    assert len(dists) > 250, f"recorder too sparse over the hold window: {len(dists)} samples"

    mean_dist = sum(dists) / len(dists)
    log(f"orbit mean radius {mean_dist:.1f} m, peak {max(dists):.1f} m, "
        f"swept {math.degrees(sweep):.0f} deg")
    # The vehicle rides the ring with pursuit lag (a little inside) plus the
    # loop phase lag (a little outside); a hover at the hold point would sit
    # near zero and a runaway pursuit far outside.
    assert 3.0 < mean_dist < 12.0, f"orbit radius off: mean {mean_dist:.1f} m from the hold point"
    assert max(dists) < 16.0, f"orbit excursion: {max(dists):.1f} m from the hold point"
    assert sweep > math.radians(270.0), f"no sustained circulation: swept {math.degrees(sweep):.0f} deg"
    assert BOX_ARM in sitl.modes(), "unexpected disarm during the orbit"


def scenario_mission_figure8(sitl, rc, fdm):
    """HOLD with the FIGURE8 pattern: bounded excursion about the hold point
    with repeated passes back through the centre."""
    boot_and_engage(sitl, rc, fdm)

    wait_for(
        "arrival at the hold point",
        lambda: fdm.distance_to_wp(0.0, 40.0) < 10.0,
        timeout=90,
        interval=1.0,
    )
    t_arrive = fdm.now_t()

    wait_for("figure-8 window elapsed", lambda: fdm.now_t() > t_arrive + 52.0,
             timeout=70, interval=2.0)
    dists, _ = hold_window_samples(fdm, 0.0, 40.0, t_arrive + 12.0, t_arrive + 52.0)
    assert len(dists) > 250, f"recorder too sparse over the hold window: {len(dists)} samples"

    # Lemniscate on an 8 m radius: lobes reach the ring, the path re-crosses
    # the centre twice per cycle (~25 s), and never leaves the hold radius.
    log(f"figure-8 peak {max(dists):.1f} m from the hold point")
    assert max(dists) < 13.0, f"figure-8 excursion: {max(dists):.1f} m from the hold point"
    assert max(dists) > 4.0, f"no pattern motion: peak {max(dists):.1f} m from the hold point"
    crossings = 0
    away = False
    for d in dists:
        if d > 5.0:
            away = True
        elif away and d < 4.0:
            crossings += 1
            away = False
    assert crossings >= 2, f"path did not re-cross the centre: {crossings} passes"
    assert BOX_ARM in sitl.modes(), "unexpected disarm during the figure-8"
    log(f"figure-8 {crossings} centre passes")


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
        # Plan swap, not rescue: the mission keeps flying (an injected
        # [fly home, land] plan) and the vehicle comes back inside the fence.
        time.sleep(3)
        modes = sitl.modes()
        assert BOX_GPSRESCUE not in modes, f"rescue engaged instead of plan swap: {modes}"
        assert BOX_AUTOPILOT in modes, f"mission dropped on breach: {modes}"
        wait_for(
            "vehicle returns inside the fence",
            lambda: fdm.distance_from_home() < 25.0,
            timeout=120,
            interval=1.0,
        )
        wait_for(
            "touchdown at home disarms",
            lambda: BOX_ARM not in sitl.modes(),
            timeout=120,
            interval=1.0,
        )
        dist = fdm.distance_from_home()
        assert fdm.model.on_ground(), f"disarmed in the air: alt {fdm.model.pos[2]:.1f} m"
        assert dist < 10.0, f"landed {dist:.1f} m from home"
        log(f"returned and landed {dist:.1f} m from home")
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
    """The geofence return must survive RX loss: with rx-loss policy CONTINUE
    the failsafe keeps flying the injected return plan while stage-2 rxfail
    values force the AUTOPILOT switch low."""
    boot_and_engage(sitl, rc, fdm)
    wait_for("mission departs toward the fence", lambda: fdm.distance_from_home() > 40.0, timeout=60)
    wait_for(
        "return leg underway (heading back inside 40 m)",
        lambda: fdm.distance_from_home() < 40.0,
        timeout=90,
        interval=0.5,
    )

    log("killing RC stream mid-return")
    rc.stop_stream()
    wait_for("failsafe active", lambda: BOX_FAILSAFE in sitl.modes())
    time.sleep(3)
    modes = sitl.modes()
    assert {BOX_FAILSAFE, BOX_AUTOPILOT} <= modes, f"return plan dropped on RX loss: {modes}"
    assert BOX_GPSRESCUE not in modes, f"unexpected rescue: {modes}"
    log("return continues through RX loss")
    wait_for(
        "lands at home under failsafe",
        lambda: BOX_ARM not in sitl.modes(),
        timeout=150,
        interval=1.0,
    )
    dist = fdm.distance_from_home()
    assert dist < 10.0, f"landed {dist:.1f} m from home"
    log(f"landed {dist:.1f} m from home under failsafe")


# --- GPS rescue scenarios -------------------------------------------------
# GPS rescue is flown as an autopilot mission (ENABLE_RESCUE_PLAN, now the
# default on flight-plan targets): on RC loss the failsafe procedure stages a
# rescue plan and flies it under FAILSAFE + AUTOPILOT. Each scenario asserts the
# safety outcome directly: engage, return, land near home, disarm, no flyaway.

RESCUE_CFG = [
    "set failsafe_procedure = GPS-RESCUE",
    # FIXED_ALT: the default MAX mode keys the return altitude to each leg's
    # own outbound peak, which varies run to run and would dominate the A/B
    # altitude comparison; MAX synthesis is unit-tested instead
    "set gps_rescue_alt_mode = FIXED_ALT",
    "set gps_rescue_return_alt = 30",     # long climb widens the ascendRate-clamp margin
    # ascendRate 1 m/s clamps the climb feedforward well under the ~2.2 m/s the
    # model reaches on this climb under the alt-hold climbRate (5 m/s), so the
    # climb rate proves ascendRate. descendRate 0.8 m/s governs the fallback
    # descent (baro-only, velocity-tracked) held under the ~1.26 m/s throttle-
    # floor descent the alt-hold climbRate would otherwise drive it to.
    "set gps_rescue_ascend_rate = 100",
    "set gps_rescue_descend_rate = 80",
    "set ap_yaw_mode = FIXED",
    "set ap_waypoint_hold_radius = 400",
    "set ap_landing_descent_rate = 200",
    "set landing_disarm_threshold = 10",   # jerk-based touchdown disarm
    "aux 3 3 3 1700 2100 0 0",   # ALTHOLD on AUX4: pilot-flown hold after the mission leg
    "aux 4 11 3 1700 2100 0 0",  # POSHOLD on AUX4
    "feature BLACKBOX",
    "set blackbox_device = VIRTUAL",       # .BFL artifact in the scenario dir
]


def fly_out_and_park(sitl, rc, fdm, dist_m):
    """Mission leg out to dist_m, then hand to pilot-held ALTHOLD+POSHOLD."""
    boot_and_engage(sitl, rc, fdm)
    wait_for(
        f"vehicle {dist_m:.0f} m out",
        lambda: fdm.distance_from_home() > dist_m,
        timeout=90,
        interval=1.0,
    )
    rc.set(7, RC_HIGH)  # AUX4: ALTHOLD + POSHOLD (pilot hold)
    rc.set(5, 1000)     # AUX2: AUTOPILOT off
    wait_for(
        "pilot hold (AUTOPILOT off, POSHOLD on)",
        lambda: (lambda m: BOX_AUTOPILOT not in m and BOX_POSHOLD in m)(sitl.modes()),
        timeout=10,
    )
    time.sleep(2.0)


def rescue_engagement_asserts(sitl, variant="B"):
    # Converged rescue: GPS rescue is flown as an autopilot mission, so it runs
    # under FAILSAFE + AUTOPILOT and never enables the legacy GPS_RESCUE box.
    wait_for(
        "rescue mission engaged (FAILSAFE + AUTOPILOT)",
        lambda: (lambda m: BOX_FAILSAFE in m and BOX_AUTOPILOT in m)(sitl.modes()),
        timeout=20,
    )
    assert BOX_GPSRESCUE not in sitl.modes(), "legacy GPS_RESCUE engaged instead of the rescue mission"


def rescue_metrics(fdm, t0, kill_dist):
    return {
        "kill_dist": kill_dist,
        "max_alt": max((s[3] for s in fdm.snapshot_history() if s[0] >= t0), default=0.0),
        "max_dist": fdm.max_distance_from_home(after_t=t0),
        "time_to_home": fdm.time_to_home(radius_m=20.0, after_t=t0),
        "touchdown": fdm.touchdown(after_t=t0),
    }


def band_descent_rate(fdm, t0, lo_alt, hi_alt):
    """Median descent rate (m/s, positive down) over an altitude band, ignoring
    the ramp-in at the top and the near-ground slowdown."""
    s = sorted(-r[6] for r in fdm.snapshot_history()
               if r[0] >= t0 and lo_alt <= r[3] <= hi_alt and r[6] < -0.1)
    return s[len(s) // 2] if s else 0.0


def assert_rescue_climb_rate(fdm, t0, variant):
    # The rescue climb feedforward is clamped to gps_rescue_ascend_rate (1 m/s).
    # The altitude P-term still drives a transient above the cap, but at a much
    # lower peak (~1.8 m/s) than the ~2.6 m/s this climb reaches under the
    # alt-hold climbRate (5 m/s): the peak shows ascendRate shaping the climb.
    peak_climb = max((s[6] for s in fdm.snapshot_history() if s[0] >= t0), default=0.0)
    log(f"[{variant}] climb rate: peak {peak_climb:.2f} m/s (ascendRate 1.0)")
    assert 0.6 <= peak_climb <= 2.25, f"climb not held to ascendRate: {peak_climb:.2f} m/s"


def scenario_rescue_ab(sitl, rc, fdm, variant="B"):
    fly_out_and_park(sitl, rc, fdm, 120.0)
    kill_dist = fdm.distance_from_home()
    t0 = fdm.now_t()
    log(f"[{variant}] killing RC {kill_dist:.0f} m out")
    rc.stop_stream()

    rescue_engagement_asserts(sitl, variant)
    wait_for(
        "returns within 20 m of home",
        lambda: fdm.distance_from_home() < 20.0,
        timeout=120,
        interval=1.0,
    )
    wait_for(
        "touchdown disarms",
        lambda: fdm.model.on_ground() and BOX_ARM not in sitl.modes(),
        timeout=120,
        interval=1.0,
    )
    m = rescue_metrics(fdm, t0, kill_dist)
    td = m["touchdown"]
    assert td is not None, "no touchdown recorded"
    td_dist = math.hypot(td[1], td[2])
    assert td_dist < 15.0, f"landed {td_dist:.1f} m from home"
    log(f"[{variant}] landed {td_dist:.1f} m from home, peak alt {m['max_alt']:.1f} m")
    m["td_dist"] = td_dist

    assert_rescue_climb_rate(fdm, t0, variant)
    return m


def scenario_rescue_fast_entry(sitl, rc, fdm, variant="B"):
    """RC lost mid-dash, the craft still running away from home at speed. It
    cannot stop inside the distance a fixed fence allows, so the rescue mission
    used to trip its own flyaway check while braking, abort, and sink where it
    was instead of returning."""
    boot_and_engage(sitl, rc, fdm)
    wait_for(
        "vehicle 35 m out and running",
        lambda: fdm.distance_from_home() > 35.0 and fdm.ground_speed() > 7.0,
        timeout=60,
        interval=0.2,
    )
    kill_dist = fdm.distance_from_home()
    entry_speed = fdm.ground_speed()
    t0 = fdm.now_t()
    log(f"[{variant}] killing RC {kill_dist:.0f} m out at {entry_speed:.1f} m/s")
    rc.stop_stream()

    rescue_engagement_asserts(sitl, variant)
    wait_for(
        "returns within 20 m of home",
        lambda: fdm.distance_from_home() < 20.0,
        timeout=180,
        interval=1.0,
    )
    wait_for(
        "touchdown disarms",
        lambda: fdm.model.on_ground() and BOX_ARM not in sitl.modes(),
        timeout=120,
        interval=1.0,
    )
    m = rescue_metrics(fdm, t0, kill_dist)
    td = m["touchdown"]
    assert td is not None, "no touchdown recorded"
    td_dist = math.hypot(td[1], td[2])
    assert td_dist < 15.0, f"landed {td_dist:.1f} m from home"
    # The overshoot is the whole point: the craft has to coast well past the
    # trigger point and still come home.
    overshoot = m["max_dist"] - kill_dist
    log(f"[{variant}] overshot {overshoot:.1f} m, landed {td_dist:.1f} m from home")
    assert overshoot > 5.0, f"entry too gentle to exercise the fence: {overshoot:.1f} m"
    m["td_dist"] = td_dist
    m["overshoot"] = overshoot
    return m


def scenario_rescue_near_home(sitl, rc, fdm, variant="B"):
    """RC lost hovering low a few metres from home. Inside gps_rescue_min_start_dist
    there is no return to fly, and climbing first only lifts the craft over
    whoever is near home: it lands where it stops."""
    boot_and_engage(sitl, rc, fdm)
    wait_for("vehicle 3 m out", lambda: fdm.distance_from_home() > 3.0, timeout=30, interval=0.1)
    rc.set(7, RC_HIGH)  # AUX4: ALTHOLD + POSHOLD (pilot hold)
    rc.set(5, 1000)     # AUX2: AUTOPILOT off, before the mission carries it away
    wait_for(
        "pilot hold (AUTOPILOT off, POSHOLD on)",
        lambda: (lambda m: BOX_AUTOPILOT not in m and BOX_POSHOLD in m)(sitl.modes()),
        timeout=10,
    )
    wait_for("settled in the hold", lambda: fdm.ground_speed() < 0.5, timeout=20, interval=0.2)
    time.sleep(2.0)
    start_dist = fdm.distance_from_home()
    start_e, start_n, start_alt = fdm.model.pos[0], fdm.model.pos[1], fdm.model.pos[2]
    assert 2.0 < start_dist < 8.0, f"held {start_dist:.1f} m from home, outside the close-range case"
    t0 = fdm.now_t()
    log(f"[{variant}] killing RC {start_dist:.1f} m from home at {start_alt:.1f} m")
    rc.stop_stream()

    rescue_engagement_asserts(sitl, variant)
    wait_for(
        "touchdown disarms",
        lambda: fdm.model.on_ground() and BOX_ARM not in sitl.modes(),
        timeout=60,
        interval=0.5,
    )
    m = rescue_metrics(fdm, t0, start_dist)
    td = m["touchdown"]
    assert td is not None, "no touchdown recorded"
    climb = m["max_alt"] - start_alt
    moved = math.hypot(td[1] - start_e, td[2] - start_n)
    log(f"[{variant}] climbed {climb:.1f} m, landed {moved:.1f} m from where RC was lost")
    assert climb < 1.0, f"climbed {climb:.1f} m before landing near home"
    assert moved < 3.0, f"landed {moved:.1f} m from where it stopped"
    m["climb"] = climb
    m["moved"] = moved
    return m


def scenario_rescue_heading_recovery(sitl, rc, fdm, variant="B", drift=False):
    """No mag, and flown out on roll alone, so the GPS course never teaches the
    IMU a heading: RC lost well clear of home. The rescue climbs level to the
    return altitude, pitches forward until the course has taught the IMU its
    heading, then flies home and lands. With drift, RC is lost mid-run just
    under the return altitude, so the climb is over while the craft is still
    sliding sideways."""
    rc.start()
    fdm.start()
    wait_for("GPS fix + RX recovery (arming flags clear)", lambda: sitl.status()["arming_flags"] == 0, timeout=40)
    sitl.acc_calibrate()
    time.sleep(2.0)
    wait_for("recalibration complete", lambda: sitl.status()["arming_flags"] == 0, timeout=20)

    rc.set(6, RC_HIGH)  # ANGLE
    for attempt in range(3):
        rc.set(4, RC_HIGH)
        try:
            wait_for("armed", lambda: BOX_ARM in sitl.modes(), timeout=8)
            break
        except AssertionError:
            if attempt == 2:
                raise
            rc.set(4, 1000)
            time.sleep(1.0)
    rc.set(2, 1600)
    rc.set(7, RC_HIGH)  # ALTHOLD + POSHOLD: without a heading, POSHOLD passes the sticks through
    climb_to = 13.5 if drift else 6.0
    wait_for("climbed clear of ground", lambda: fdm.model.pos[2] > climb_to, timeout=30)
    rc.set(2, 1300)

    # Past 10 deg of roll the course teaches the IMU nothing, so a sideways run leaves the heading unknown.
    rc.set(0, 1900)
    wait_for("40 m out sideways", lambda: fdm.distance_from_home() > 40.0, timeout=30, interval=0.1)
    if not drift:
        rc.set(0, 1100)
        wait_for("braked", lambda: fdm.model.vel[0] < 1.0, timeout=15, interval=0.1)
        rc.set(0, RC_MID)
        wait_for("stopped", lambda: fdm.ground_speed() < 1.0, timeout=30, interval=0.5)
    assert sitl.debug_values()[4] == 1, "the IMU learnt its heading on the way out"

    kill_dist = fdm.distance_from_home()
    start_alt = fdm.model.pos[2]
    t0 = fdm.now_t()
    log(f"[{variant}] killing RC {kill_dist:.0f} m out at {start_alt:.1f} m, "
        f"{fdm.ground_speed():.1f} m/s, heading unknown")
    rc.stop_stream()

    rescue_engagement_asserts(sitl, variant)
    wait_for("heading learnt", lambda: sitl.debug_values()[4] == 0, timeout=60, interval=0.2)
    learnt_t = fdm.now_t()
    alt_at_learnt = fdm.model.pos[2]
    # The IMU against the craft's true nose while it settles the heading on the course.
    heading_err = 0.0
    while fdm.now_t() < learnt_t + 1.0:
        heading_err = max(heading_err, abs((sitl.yaw_deg() - fdm.heading_deg() + 180.0) % 360.0 - 180.0))
        time.sleep(0.1)
    wait_for(
        "returns within 20 m of home",
        lambda: fdm.distance_from_home() < 20.0,
        timeout=120,
        interval=1.0,
    )
    home_t = fdm.now_t()
    wait_for(
        "touchdown disarms",
        lambda: fdm.model.on_ground() and BOX_ARM not in sitl.modes(),
        timeout=120,
        interval=1.0,
    )
    m = rescue_metrics(fdm, t0, kill_dist)
    td = m["touchdown"]
    assert td is not None, "no touchdown recorded"
    td_dist = math.hypot(td[1], td[2])
    hist = fdm.snapshot_history()
    # Level until the climb is done: a pitch-forward low down picks up speed well before 12.5 m.
    climb_speed = max((math.hypot(s[4], s[5]) for s in hist if s[0] >= t0 and s[3] < 12.5), default=0.0)
    # Without a compass the heading is only kept at speed: from learning it to nearly home, never stopped.
    return_speed = min((math.hypot(s[4], s[5]) for s in hist if learnt_t <= s[0] <= home_t), default=0.0)
    pitch_forward = next((s for s in hist if s[0] >= t0 and s[8] > 20.0), None)
    assert pitch_forward is not None, "never pitched forward"
    pitch_forward_speed = math.hypot(pitch_forward[4], pitch_forward[5])
    log(f"[{variant}] pitched forward {pitch_forward[0] - t0:.1f} s in at {pitch_forward_speed:.1f} m/s, "
        f"heading learnt {learnt_t - t0:.0f} s in at {alt_at_learnt:.1f} m, IMU off the nose by up to "
        f"{heading_err:.0f} deg, climb speed {climb_speed:.1f} m/s, slowest after {return_speed:.1f} m/s, "
        f"furthest {m['max_dist']:.0f} m, landed {td_dist:.1f} m from home")
    assert pitch_forward_speed < 1.5, f"pitched forward still drifting {pitch_forward_speed:.1f} m/s"
    assert heading_err < 10.0, f"learnt a heading {heading_err:.0f} deg off the nose"
    assert climb_speed < 3.0, f"moving {climb_speed:.1f} m/s before the climb was done"
    assert return_speed > 2.5, f"slowed to {return_speed:.1f} m/s after learning the heading"
    assert m["max_dist"] < kill_dist + 80.0, f"heading-recovery excursion ran away: {m['max_dist']:.0f} m"
    assert td_dist < 15.0, f"landed {td_dist:.1f} m from home"
    m["td_dist"] = td_dist
    return m


def scenario_rescue_near_home_no_heading(sitl, rc, fdm, variant="B"):
    """No mag and no forward flight, so no heading, and RC lost hovering over
    home. Position hold cannot run without a heading, so the rescue leaves the
    landing to the failsafe's altitude-only descent, which lands the craft
    where it is straight away rather than after the plan stalls."""
    rc.start()
    fdm.start()
    wait_for("GPS fix + RX recovery (arming flags clear)", lambda: sitl.status()["arming_flags"] == 0, timeout=40)
    sitl.acc_calibrate()
    time.sleep(2.0)
    wait_for("recalibration complete", lambda: sitl.status()["arming_flags"] == 0, timeout=20)

    rc.set(6, RC_HIGH)  # ANGLE
    for attempt in range(3):
        rc.set(4, RC_HIGH)
        try:
            wait_for("armed", lambda: BOX_ARM in sitl.modes(), timeout=8)
            break
        except AssertionError:
            if attempt == 2:
                raise
            rc.set(4, 1000)
            time.sleep(1.0)
    rc.set(2, 1600)
    rc.set(7, RC_HIGH)  # ALTHOLD + POSHOLD hover (switch must be off at arm time)
    wait_for("climbed clear of ground", lambda: fdm.model.pos[2] > 6.0, timeout=20)
    rc.set(2, 1300)
    time.sleep(2.0)

    kill_dist = fdm.distance_from_home()
    start_alt = fdm.model.pos[2]
    t0 = fdm.now_t()
    log(f"[{variant}] killing RC at hover, no heading, {kill_dist:.1f} m from home at {start_alt:.1f} m")
    rc.stop_stream()

    wait_for(
        "failsafe landing, no plan to fly (FAILSAFE, no AUTOPILOT)",
        lambda: (lambda m: BOX_FAILSAFE in m and BOX_AUTOPILOT not in m)(sitl.modes()),
        timeout=20,
    )
    wait_for(
        "touchdown disarms",
        lambda: fdm.model.on_ground() and BOX_ARM not in sitl.modes(),
        timeout=60,
        interval=0.5,
    )
    m = rescue_metrics(fdm, t0, kill_dist)
    td = m["touchdown"]
    assert td is not None, "no touchdown recorded"
    td_dist = math.hypot(td[1], td[2])
    climb = m["max_alt"] - start_alt
    down_s = td[0] - t0
    log(f"[{variant}] climbed {climb:.1f} m, down in {down_s:.0f} s, {td_dist:.1f} m from home")
    assert climb < 1.0, f"climbed {climb:.1f} m before landing near home"
    assert down_s < 25.0, f"took {down_s:.0f} s to land"
    assert td_dist < 5.0, f"landed {td_dist:.1f} m from home"
    m["td_dist"] = td_dist
    return m


def scenario_rescue_gps_loss(sitl, rc, fdm, variant="B"):
    fly_out_and_park(sitl, rc, fdm, 120.0)
    kill_dist = fdm.distance_from_home()
    t0 = fdm.now_t()
    log(f"[{variant}] killing RC {kill_dist:.0f} m out")
    rc.stop_stream()

    rescue_engagement_asserts(sitl, variant)
    wait_for(
        "return underway (30 m closer)",
        lambda: fdm.distance_from_home() < kill_dist - 30.0,
        timeout=90,
        interval=1.0,
    )
    loss_dist = fdm.distance_from_home()
    log(f"[{variant}] GPS dark {loss_dist:.0f} m out")
    fdm.gps_valid = False

    # A: legacy emergency descent (baro only). B: mission aborts on estimator
    # loss and failsafe degrades to the baro auto-landing. Both must get down
    # and disarm without flying away.
    wait_for(
        "descends and disarms without GPS",
        lambda: fdm.model.on_ground() and BOX_ARM not in sitl.modes(),
        timeout=180,
        interval=1.0,
    )
    m = rescue_metrics(fdm, t0, kill_dist)
    assert m["max_dist"] < kill_dist + 40.0, f"flew away after GPS loss: {m['max_dist']:.0f} m"
    log(f"[{variant}] down and disarmed after GPS loss")
    return m


def scenario_rescue_switch_descent(sitl, rc, fdm, variant="B"):
    """Switch-invoked rescue (no RC loss): the pilot flips BOXGPSRESCUE. The plan
    flies as an autopilot mission, then GPS goes dark mid-return. Because the
    SWITCH (not failsafe) invoked it, the aborted plan degrades to a controlled
    altitude-only descent and disarms - never entering FAILSAFE."""
    fly_out_and_park(sitl, rc, fdm, 120.0)   # out, then pilot hold; RC stays live
    kill_dist = fdm.distance_from_home()
    t0 = fdm.now_t()
    log(f"[{variant}] flipping GPS-RESCUE switch {kill_dist:.0f} m out")
    rc.set(7, RC_LOW)    # hand over: drop pilot ALTHOLD+POSHOLD
    rc.set(8, RC_HIGH)   # AUX5: BOXGPSRESCUE

    # The switch flies the rescue as an autopilot mission: AUTOPILOT engages, never
    # the failsafe path or the legacy GPS_RESCUE box.
    wait_for(
        "rescue plan engaged via switch (AUTOPILOT, no FAILSAFE)",
        lambda: (lambda m: BOX_AUTOPILOT in m and BOX_FAILSAFE not in m)(sitl.modes()),
        timeout=20,
    )
    assert BOX_GPSRESCUE not in sitl.modes(), "legacy GPS_RESCUE engaged instead of the plan"

    wait_for(
        "return underway (30 m closer)",
        lambda: fdm.distance_from_home() < kill_dist - 30.0,
        timeout=90,
        interval=1.0,
    )
    loss_dist = fdm.distance_from_home()
    log(f"[{variant}] GPS dark {loss_dist:.0f} m out (switch still held)")
    fdm.gps_valid = False

    # Plan aborts on estimator loss; the switch invoker degrades to the baro-only
    # descent (not failsafe) and disarms without flying away.
    wait_for(
        "descends and disarms without GPS",
        lambda: fdm.model.on_ground() and BOX_ARM not in sitl.modes(),
        timeout=180,
        interval=1.0,
    )
    assert BOX_FAILSAFE not in sitl.modes(), "entered FAILSAFE on a switch-invoked rescue"
    m = rescue_metrics(fdm, t0, kill_dist)
    assert m["max_dist"] < kill_dist + 40.0, f"flew away after GPS loss: {m['max_dist']:.0f} m"
    log(f"[{variant}] down and disarmed via switch-fallback descent")

    # The climb ran under ascendRate; the baro-only fallback descent runs at
    # gps_rescue_descend_rate (0.8 m/s) - the alt-hold climbRate (5 m/s) would
    # drive it to the ~1.26 m/s throttle floor. Sample the descent near ground,
    # where the failsafe-landing profile's altitude scaling has decayed to ~1x.
    assert_rescue_climb_rate(fdm, t0, variant)
    descent = band_descent_rate(fdm, t0, 2.0, 7.0)
    log(f"[{variant}] fallback descent: {descent:.2f} m/s (descendRate 0.8)")
    assert 0.5 <= descent <= 1.1, f"fallback descent not held to descendRate: {descent:.2f} m/s"
    return m


def scenario_mission_vert_rate(sitl, rc, fdm):
    """A leg that states a vertical rate climbs at that rate, not at the alt hold
    climb rate, and the altitude walks up instead of stepping."""
    boot_and_engage(sitl, rc, fdm)

    wait_for("climb under way (8 m)", lambda: fdm.model.pos[2] > 8.0, timeout=60, interval=0.5)
    startT = time.monotonic()
    startAlt = fdm.model.pos[2]
    wait_for("climbs through 24 m", lambda: fdm.model.pos[2] > 24.0, timeout=90, interval=0.5)
    rateMps = (fdm.model.pos[2] - startAlt) / (time.monotonic() - startT)
    # the leg states 1.0 m/s; the alt hold climb rate this would otherwise take is 5 m/s
    assert 0.7 <= rateMps <= 1.5, f"leg climb rate off target: {rateMps:.2f} m/s"
    assert BOX_ARM in sitl.modes(), "unexpected disarm during the rate-limited climb"
    log(f"climbed at {rateMps:.2f} m/s against the leg's commanded 1.0 m/s")


def scenario_mission_face_target(sitl, rc, fdm):
    """FACE_TARGET: engaged with the nose 180 deg off the leg, the craft holds
    station until the nose is on the waypoint, then flies it."""
    boot_and_engage(sitl, rc, fdm)

    def headingErr():
        return abs((fdm.heading_deg() - 0.0 + 180.0) % 360.0 - 180.0)

    drift = []

    def alignedYet():
        drift.append(fdm.distance_from_home())
        return headingErr() < 30.0

    wait_for("nose swings onto the leg (~000)", alignedYet, timeout=25, interval=0.2)
    heldM = max(drift)
    assert heldM < 10.0, f"translated {heldM:.1f} m before the nose came round"
    log(f"held station within {heldM:.1f} m while rotating")

    wait_for("departs once aligned", lambda: fdm.distance_from_home() > 20.0, timeout=45, interval=0.5)
    wait_for(
        "reaches the waypoint (ground truth)",
        lambda: fdm.distance_to_wp(0.0, 90.0) < 12.0,
        timeout=120,
        interval=1.0,
    )
    assert BOX_ARM in sitl.modes(), "unexpected disarm on the face-the-target leg"


def scenario_mission_face_next(sitl, rc, fdm):
    """FACE_NEXT: flying the first leg north, the nose points at the second
    waypoint (90 m north, 90 m east) rather than along the course."""
    boot_and_engage(sitl, rc, fdm)

    samples = []

    def midLeg():
        north = fdm.model.pos[1]
        if 30.0 < north < 70.0:
            bearing = math.degrees(math.atan2(90.0 - fdm.model.pos[0], 90.0 - north)) % 360.0
            samples.append((fdm.heading_deg(), bearing))
        return north > 70.0

    wait_for("flies the first leg", midLeg, timeout=90, interval=0.5)
    assert len(samples) >= 5, f"leg too short to sample: {len(samples)}"
    errs = [abs((h - b + 180.0) % 360.0 - 180.0) for h, b in samples]
    avgErr = sum(errs) / len(errs)
    courseErrs = [abs((h - 0.0 + 180.0) % 360.0 - 180.0) for h, _ in samples]
    avgCourseErr = sum(courseErrs) / len(courseErrs)
    assert avgErr < 35.0, f"nose did not track the next waypoint: {avgErr:.0f} deg off its bearing"
    assert avgCourseErr > 25.0, f"nose tracked the course, not the next waypoint: {avgCourseErr:.0f} deg off course"
    log(f"nose held the next waypoint's bearing ({avgErr:.0f} deg off it, {avgCourseErr:.0f} deg off course)")


WING_CFG = [
    "mixer FLYING_WING",
    "aux 3 58 3 1700 2100 0 0",   # LAUNCH on AUX4
    "set ap_wing_land_launch_height = 0",   # armed on the ground, then picked up and thrown
]
WING_NO_MAG_CFG = ["set mag_hardware = NONE"]
WING_MOTOR_STOP_CFG = ["feature MOTOR_STOP"]

# Firmware defaults the checks are measured against
WING_CRUISE_SPEED = 15.0          # m/s, ap_wing_cruise_speed
WING_MAX_BANK_DEG = 35.0          # ap_wing_max_bank
WING_ROLL_SLEW_DPS = 45.0         # the guidance's bank slew
WING_ROLL_LAG_S = 0.25            # the attitude following the bank demand
WING_L1_PERIOD_S = 13.0           # ap_wing_l1_period
WING_L1_DAMPING = 0.75            # ap_wing_l1_damping
WING_L1_MIN_M = 10.0
WING_ARRIVAL_RADIUS_M = 10.0
WING_LOITER_RADIUS = 60.0         # ap_wing_loiter_radius
WING_FLARE_HEIGHT_M = 3.0         # ap_wing_land_flare_height
WING_LEVEL_HEIGHT_M = 3.0 * WING_FLARE_HEIGHT_M   # below this a landing descent rolls the wings level
WING_APPROACH_ALT_M = 30.0        # ap_wing_land_approach_alt
WING_GPS_LOST_S = 2.5 + 2.0       # the GPS receive timeout, then the position estimate's measurement timeout
WING_FAILSAFE_DELAY_S = 1.0       # failsafe_delay in base_config

# The plant's airspeed in level flight on the default throttle schedule
WING_CRUISE_AIRSPEED = 16.3

# Pass criteria shared by the wing scenarios
WING_ALT_HOLD_M = 3.0             # alt hold's error from its target once settled
WING_STALL_LIMIT_S = 0.5          # the longest airborne stall
WING_CONTACT_SINK = 1.0           # m/s
WING_CONTACT_ROLL_DEG = 5.0
WING_UNKNOWN_GROUND_LOW_BANK_DEG = 10.0       # over ground of unknown height a descent circles at most this near home's
WING_UNKNOWN_GROUND_BANK_DEG = 15.0           # ground, and this above it
WING_UNKNOWN_GROUND_CONTACT_ROLL_DEG = 12.0
WING_TURN_BANK_FRACTION = 0.8                 # of a bank limit, the guidance plans its turns on


def wing_circle_m(bank_deg):
    """The radius of the circle the guidance plans at the cruise speed with a bank limit."""
    return WING_CRUISE_SPEED ** 2 / (GRAVITY * math.tan(math.radians(WING_TURN_BANK_FRACTION * bank_deg)))
WING_CONTACT_PITCH_DEG = (0.0, 12.0)   # nose up
WING_CONTACT_AIRSPEED = 1.5 * WING_V_STALL
WING_TRACK_TOLERANCE_DEG = 10.0   # the ground track at the contact against the landing heading
WING_TOUCHDOWN_ALONG_M = (-20.0, 40.0)
WING_TOUCHDOWN_ACROSS_M = 8.0
WING_MOTOR_IDLE = 0.1
WING_MOTOR_CUT_S = 0.5            # the longest the motor may run above idle after the contact
WING_STOP_TO_DISARM_S = 3.0

WING_THROW_SPEED = 14.0           # m/s over the ground
WING_LAUNCH_STICK = 1800
WING_CLIMB_STICK = 1200           # the throttle stick after the throw: a hold must not follow it


def wing_l1_m(groundspeed):
    """The guidance's look-ahead distance at a groundspeed."""
    return max(WING_L1_DAMPING * WING_L1_PERIOD_S * groundspeed / math.pi, WING_L1_MIN_M)


def wing_turn_distance_m(turn_deg, groundspeed):
    """How far short of a corner the leg planner starts the turn."""
    return wing_l1_m(groundspeed) * min(abs(turn_deg) / 90.0, 1.0)


def wing_groundspeed(wind, track_deg, airspeed=WING_CRUISE_AIRSPEED):
    """Groundspeed along a track at an airspeed, crabbed into the wind."""
    ue, un = math.sin(math.radians(track_deg)), math.cos(math.radians(track_deg))
    along = wind[0] * ue + wind[1] * un
    across = wind[0] * un - wind[1] * ue
    return along + math.sqrt(max(0.0, airspeed * airspeed - across * across))


def require(values, what):
    if not values:
        raise AssertionError(f"no samples {what}")
    return values


def mean(values, what):
    return sum(require(values, what)) / len(values)


def rms(values, what):
    return math.sqrt(sum(v * v for v in require(values, what)) / len(values))


class WingSample(collections.namedtuple("WingSample", "t e n u roll vu yaw_rate gs debug")):
    """Ground truth at t (ENU metres, bank in degrees, climb and yaw rates, groundspeed), and the FC's debug values
    when the trace reads them. The debug accessors follow the scenario's debug mode: FLIGHT_PLAN (nav state, abort,
    leg) or AUTOPILOT_LANDING (phase, go-arounds, cause)."""

    __slots__ = ()

    @property
    def nav_state(self):
        return self.debug[0]

    @property
    def abort(self):
        return self.debug[1]

    @property
    def leg(self):
        return self.debug[2]

    @property
    def phase(self):
        return self.debug[0]

    @property
    def attempts(self):
        return self.debug[1]

    @property
    def cause(self):
        return self.debug[2]


class WingTrace:
    """Samples a wing's flight, with the FC's debug values when debug is set. A crash fails the scenario at once,
    and so does a flight plan abort when abort_check is set (debug mode FLIGHT_PLAN)."""

    def __init__(self, sitl, fdm, debug=False, abort_check=False):
        self.sitl = sitl
        self.fdm = fdm
        self.debug = debug or abort_check
        self.abort_check = abort_check
        self.samples = []

    def sample(self):
        m = self.fdm.model
        if m.crashed:
            last = self.samples[-1] if self.samples else None
            raise AssertionError(f"crashed: {m.crash_reason}" + (f" (last debug {last.debug})" if last else ""))
        s = WingSample(time.monotonic(), m.pos[0], m.pos[1], m.pos[2], math.degrees(m.roll), m.vel[2], m.rates[2],
                       math.hypot(m.vel[0], m.vel[1]), self.sitl.debug_values() if self.debug else None)
        assert not self.abort_check or s.abort == 0, f"flight plan aborted (reason {s.abort}) at leg {s.leg}"
        self.samples.append(s)
        return s

    def fly(self, seconds, interval=0.1, each=None):
        """Fly for seconds, calling each(t) before every sample; returns the samples taken."""
        start = len(self.samples)
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < seconds:
            if each:
                each(t)
            self.sample()
            time.sleep(interval)
        return self.samples[start:]

    def fly_until(self, done, timeout, what, interval=0.1, each=None):
        """Fly until done(sample), calling each(sample) on every sample first; returns the sample that was done."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            s = self.sample()
            if each:
                each(s)
            if done(s):
                return s
            time.sleep(interval)
        last = self.samples[-1]
        raise AssertionError(f"timeout waiting for {what} (last debug {last.debug}, {last.u:.1f} m up)")


def wing_stall_failures(model, longest_s=None, where="after the launch"):
    longest_s = model.longest_stall_s if longest_s is None else longest_s
    log(f"longest airborne stall {where} {longest_s:.2f} s")
    return [f"stalled for {longest_s:.1f} s {where}"] if longest_s > WING_STALL_LIMIT_S else []


def wing_launched(fdm):
    """The launch has handed over: its stalls go on their own record, which the launch scenarios assert on, and
    the course it flew, which a landing with no heading of its own takes, is noted."""
    fdm.launch_stall_s = fdm.model.take_stall_record()
    fdm.launch_course_deg = math.degrees(math.atan2(fdm.model.vel[0], fdm.model.vel[1])) % 360.0
    log(f"longest airborne stall in the launch {fdm.launch_stall_s:.2f} s; launched on a "
        f"{fdm.launch_course_deg:.0f} deg course")


def assert_no_failures(failures):
    assert not failures, "; ".join(failures)


def wing_throw(sitl, rc, fdm, armed_at_m=0.0, not_yet=frozenset()):
    """Arm in LAUNCH on the ground (or armed_at_m up in the hand), pick the airframe up nose up, throttle up and
    throw it. The modes in not_yet must stay off until the throw."""
    fdm.model.hold_in_hand(pitch_deg=0.0, altitude_m=armed_at_m)   # level for the accelerometer recalibration
    rc.set(7, RC_HIGH)                      # AUX4: LAUNCH
    boot_and_arm(sitl, rc, fdm)
    wait_for("LAUNCH mode", lambda: BOX_LAUNCH in sitl.modes(), timeout=5)
    assert not not_yet & sitl.modes(), "engaged before the launch"
    fdm.model.hold_in_hand(pitch_deg=8.0, altitude_m=1.8)
    rc.set(2, WING_LAUNCH_STICK)            # throttle up: motor idle, then wait for the throw
    time.sleep(6.0)                         # the estimate settles on the nose-up hold
    assert not not_yet & sitl.modes(), "engaged in the thrower's hand"
    fdm.model.hand_launch(speed_ms=WING_THROW_SPEED)


def wing_hand_launch(sitl, rc, fdm):
    """Arm in LAUNCH + ANGLE, throw the airframe by hand, and wait for the
    launch to hand the climb-out back to the sticks."""
    wing_throw(sitl, rc, fdm)
    wait_for(
        "launch handed back (airborne, LAUNCH mode cleared)",
        lambda: BOX_LAUNCH not in sitl.modes() and fdm.model.pos[2] > 5.0,
        timeout=20,
    )
    wing_launched(fdm)


def wing_hold_height(rc, model, target_m):
    """The pilot holds height on the elevator (ANGLE: pitch stick forward = nose down)."""
    stick = 1500 + 15.0 * (model.pos[2] - target_m) + 30.0 * model.vel[2]
    rc.set(1, int(max(1250, min(1750, stick))))


def scenario_wing_turn(sitl, rc, fdm):
    """A minute of sustained coordinated turn in ANGLE. The accelerometer reads
    straight down the body Z axis in a coordinated turn whatever the bank, so
    unless the turn's centripetal acceleration is accounted for the estimate
    creeps towards level and ANGLE banks the aircraft ever harder to compensate."""
    wing_hand_launch(sitl, rc, fdm)
    t0 = time.monotonic()
    samples = []
    while (t := time.monotonic() - t0) < 75.0:
        wing_hold_height(rc, fdm.model, 30.0)
        if t >= 5.0:
            rc.set(0, 1780)   # ~20 deg of bank in ANGLE
        if t >= 15.0:
            est_roll, est_pitch = sitl.attitude()
            samples.append((math.degrees(fdm.model.roll), est_roll, -math.degrees(fdm.model.pitch), -est_pitch))
        assert not fdm.model.crashed, f"crashed {t:.0f} s into the turn: {fdm.model.crash_reason}"
        time.sleep(0.2)

    true_bank = mean([s[0] for s in samples], "in the turn")
    roll_errs = [s[1] - s[0] for s in samples]
    pitch_errs = [s[3] - s[2] for s in samples]
    max_roll_err = max(abs(e) for e in roll_errs)
    max_pitch_err = max(abs(e) for e in pitch_errs)
    roll_rms, pitch_rms = rms(roll_errs, "of roll"), rms(pitch_errs, "of pitch")
    log(f"turn: true bank {true_bank:.1f} deg over {len(samples)} samples; roll error rms {roll_rms:.1f} max "
        f"{max_roll_err:.1f} deg; pitch error rms {pitch_rms:.1f} max {max_pitch_err:.1f} deg, mean "
        f"{mean(pitch_errs, 'of pitch'):+.1f} deg (estimate minus true, nose up)")
    failures = wing_stall_failures(fdm.model)
    if not 15.0 < true_bank < 30.0:
        failures.append(f"the turn was not flown at a sustained bank: {true_bank:.1f} deg")
    if roll_rms >= 1.5 or max_roll_err >= 4.0:
        failures.append("roll estimate wandered off the true bank")
    if pitch_rms >= 1.5 or max_pitch_err >= 4.0:
        failures.append("pitch estimate wandered off the true pitch")
    assert_no_failures(failures)


WING_ALTHOLD_CFG = [
    *WING_CFG,
    "aux 4 3 4 1700 2100 0 0",   # ALTHOLD on AUX5
    "set debug_mode = AUTOPILOT_CLIMB",
]


def wing_bank_stick(rc, model, target_deg, state):
    """Trim the roll stick until the airframe holds target_deg of bank in ANGLE."""
    stick = state.get("stick", 1800.0) + 0.3 * (target_deg - math.degrees(model.roll))
    state["stick"] = max(1000.0, min(2000.0, stick))
    rc.set(0, int(state["stick"]))


def wing_engage_althold(sitl, rc, fdm):
    rc.set(1, RC_MID)
    rc.set(8, RC_HIGH)    # AUX5: ALTHOLD
    wait_for("ALTHOLD mode", lambda: BOX_ALTHOLD in sitl.modes(), timeout=5)
    return fdm.model.pos[2]


def after_s(samples, seconds):
    """The samples from seconds after the first."""
    return [s for s in require(samples, "flown") if s.t - samples[0].t >= seconds]


def scenario_wing_althold(sitl, rc, fdm):
    """Pitch stick commands the climb rate, centred it holds the altitude, the throttle stick is
    ignored, and a sustained bank holds height."""
    wing_hand_launch(sitl, rc, fdm)
    trace = WingTrace(sitl, fdm)
    trace.fly(4.0, each=lambda t: wing_hold_height(rc, fdm.model, 30.0))

    h0 = wing_engage_althold(sitl, rc, fdm)
    errs = [abs(s.u - h0) for s in after_s(trace.fly(25.0), 5.0)]
    log(f"level hold: entry {h0:.1f} m, mean |err| {mean(errs, 'level'):.2f} max {max(errs):.2f} m")
    failures = [f"level hold wandered {max(errs):.1f} m"] if max(errs) >= WING_ALT_HOLD_M else []

    for name, stick, sign in (("climb", 1000, 1.0), ("descent", 2000, -1.0)):
        rc.set(1, stick)
        start = fdm.model.pos[2]
        trace.fly(10.0)
        rate = (fdm.model.pos[2] - start) / 10.0
        rc.set(1, RC_MID)
        released = fdm.model.pos[2]
        overshoot = max(abs(s.u - released) for s in trace.fly(15.0))
        log(f"{name}: {rate:+.2f} m/s at full stick, within {overshoot:.2f} m of the release altitude")
        if not 1.5 <= sign * rate <= 2.5:
            failures.append(f"{name} rate {rate:+.2f} m/s at full stick")
        if overshoot >= 4.0:
            failures.append(f"{name} overshot the release altitude by {overshoot:.1f} m")

    h1 = fdm.model.pos[2]
    bank = {}
    banks = []

    def hold_bank(t):
        wing_bank_stick(rc, fdm.model, 30.0, bank)
        if t >= 5.0:
            banks.append(math.degrees(fdm.model.roll))

    turn = trace.fly(35.0, each=hold_bank)
    rc.set(0, RC_MID)
    errs = [abs(s.u - h1) for s in after_s(turn, 5.0)]
    mean_bank = mean(banks, "in the turn")
    log(f"{mean_bank:.1f} deg bank: mean |err| {mean(errs, 'in the turn'):.2f} max {max(errs):.2f} m, "
        f"final {turn[-1].u - h1:+.2f} m")
    if not 27.0 < mean_bank < 33.0:
        failures.append(f"the turn was flown at {mean_bank:.1f} deg of bank")
    if max(errs) >= WING_ALT_HOLD_M:
        failures.append(f"turn lost {max(errs):.1f} m")

    trace.fly(10.0)
    h2 = fdm.model.pos[2]
    rc.set(2, RC_LOW)
    motor = []
    low = trace.fly(10.0, each=lambda t: motor.append(fdm.motors.motors[0]))
    errs = [abs(s.u - h2) for s in low]
    log(f"throttle stick low: max |err| {max(errs):.2f} m, motor {min(motor):.2f}..{max(motor):.2f}")
    if BOX_ALTHOLD not in sitl.modes():
        failures.append("alt hold dropped with the throttle stick low")
    if max(errs) >= WING_ALT_HOLD_M:
        failures.append(f"throttle stick moved the hold {max(errs):.1f} m")
    if min(motor) <= WING_MOTOR_IDLE:
        failures.append(f"motor fell to {min(motor):.2f} with the throttle stick low")
    assert_no_failures(failures + wing_stall_failures(fdm.model))


def wing_launch_into_hold(sitl, rc, fdm, channel, boxes):
    """Arm with LAUNCH and a hold selected; the launch hands the climb-out straight to the hold.
    The throttle stick drops away after the throw, so any moment the motor follows it shows up.
    Returns where the handover happened and the failures."""
    rc.set(channel, RC_HIGH)                # selected before arming
    wing_throw(sitl, rc, fdm, not_yet=boxes)
    time.sleep(0.5)
    rc.set(2, WING_CLIMB_STICK)
    fdm.motors.trace = []

    trace = WingTrace(sitl, fdm)

    def handed_over():
        trace.sample()
        return BOX_LAUNCH not in (modes := sitl.modes()) and boxes <= modes

    wait_for("launch handed to the hold", handed_over, timeout=20, interval=0.05)
    wing_launched(fdm)
    handover_t = time.monotonic()
    entry = (fdm.model.pos[0], fdm.model.pos[1])
    h0 = fdm.model.pos[2]
    steps = []
    last = math.degrees(fdm.model.pitch)

    def watch_pitch(t):
        nonlocal last
        now = math.degrees(fdm.model.pitch)
        steps.append(abs(now - last))
        last = now

    after = trace.fly(20.0, interval=0.05, each=watch_pitch)
    motor = [m for t, m in fdm.motors.trace if abs(t - handover_t) < 1.0]
    fdm.motors.trace = None
    overshoot = max(s.u for s in after) - h0
    settle = max(abs(s.u - h0) for s in after_s(after, 10.0))
    motor_step = max(abs(b - a) for a, b in zip(motor, motor[1:]))
    log(f"handover at {h0:.1f} m: overshoot {overshoot:.2f} m, largest 50 ms pitch step {max(steps):.2f} deg, "
        f"settled within {settle:.2f} m; motor over the handover {min(motor):.3f}..{max(motor):.3f}, "
        f"largest step {motor_step:.3f} over {len(motor)} packets")
    failures = []
    if overshoot >= 8.0:
        failures.append(f"overshot the handover altitude by {overshoot:.1f} m")
    if settle >= WING_ALT_HOLD_M:
        failures.append(f"{settle:.1f} m off the handover altitude from 10 s after it")
    if max(steps) >= 10.0:
        failures.append(f"pitch stepped {max(steps):.1f} deg at the handover")
    if motor_step >= 0.05:
        failures.append(f"motor stepped {motor_step:.2f} at the handover")
    return entry, failures + wing_stall_failures(fdm.model, fdm.launch_stall_s, "in the launch")


def scenario_wing_launch_into_althold(sitl, rc, fdm):
    """Armed with LAUNCH and ALTHOLD selected, the launch hands the climb-out straight to alt hold."""
    _, failures = wing_launch_into_hold(sitl, rc, fdm, 8, {BOX_ALTHOLD})   # AUX5: ALTHOLD
    assert_no_failures(failures + wing_stall_failures(fdm.model))


WING_POSHOLD_CFG = [
    *WING_ALTHOLD_CFG,
    "aux 5 11 5 1700 2100 0 0",  # POSHOLD on AUX6
    "set debug_mode = AUTOPILOT_GUIDANCE",
]


def fit_circle(points):
    """Least-squares circle through (x, y) points: ((cx, cy), r)."""
    n = len(require(points, "to fit a circle to"))
    sx = sum(x for x, _ in points)
    sy = sum(y for _, y in points)
    sxx = sum(x * x for x, _ in points)
    syy = sum(y * y for _, y in points)
    sxy = sum(x * y for x, y in points)
    bx = -sum((x * x + y * y) * x for x, y in points)
    by = -sum((x * x + y * y) * y for x, y in points)
    b1 = -sum(x * x + y * y for x, y in points)
    m = [[sxx, sxy, sx, bx], [sxy, syy, sy, by], [sx, sy, n, b1]]
    for i in range(3):
        for j in range(i + 1, 3):
            f = m[j][i] / m[i][i]
            m[j] = [a - f * b for a, b in zip(m[j], m[i])]
    sol = [0.0, 0.0, 0.0]
    for i in (2, 1, 0):
        sol[i] = (m[i][3] - sum(m[i][k] * sol[k] for k in range(i + 1, 3))) / m[i][i]
    cx, cy = -sol[0] / 2.0, -sol[1] / 2.0
    return (cx, cy), math.sqrt(max(0.0, cx * cx + cy * cy - sol[2]))


def radius_error(s, centre, radius=WING_LOITER_RADIUS):
    return abs(math.hypot(s.e - centre[0], s.n - centre[1]) - radius)


WING_LOITER_CENTRE_M = 5.0        # the fitted circle's centre from the entry point where the bank limit allows it


def wing_loiter_stats(samples, centre, h0):
    radius_errs = [radius_error(s, centre) for s in require(samples, "on the loiter")]
    fitted, fitted_r = fit_circle([(s.e, s.n) for s in samples])
    # at the bank limit the peak groundspeed turns on no tighter than gs^2 / (g tan(bank)): the circle stretches by
    # that radius less the loiter's, and the fitted centre moves by at most the stretch
    tightest = max(s.gs for s in samples) ** 2 / (GRAVITY * math.tan(math.radians(WING_MAX_BANK_DEG)))
    return {
        "mean": mean(radius_errs, "on the loiter"),
        "max": max(radius_errs),
        "alt": max(abs(s.u - h0) for s in samples),
        "yaw_rate": mean([s.yaw_rate for s in samples], "on the loiter"),
        "centre_off": math.hypot(fitted[0] - centre[0], fitted[1] - centre[1]),
        "centre_limit": max(WING_LOITER_CENTRE_M, tightest - WING_LOITER_RADIUS),
        "fitted_r": fitted_r,
    }


def wing_engage_poshold(sitl, rc, fdm):
    rc.set(1, RC_MID)
    rc.set(9, RC_HIGH)    # AUX6: POSHOLD, which brings ALTHOLD with it
    wait_for("POSHOLD and ALTHOLD", lambda: {BOX_POSHOLD, BOX_ALTHOLD} <= sitl.modes(), timeout=5, interval=0.02)
    return (fdm.model.pos[0], fdm.model.pos[1]), fdm.model.pos[2]


def wing_loiter(sitl, rc, fdm, settle_s=30.0, window_s=60.0):
    """Launch, engage POSHOLD from level flight and fly the loiter; returns the entry point, the entry
    altitude, the trace and the stats over the window after settle_s."""
    wing_hand_launch(sitl, rc, fdm)
    trace = WingTrace(sitl, fdm)
    trace.fly(4.0, each=lambda t: wing_hold_height(rc, fdm.model, 30.0))
    entry, h0 = wing_engage_poshold(sitl, rc, fdm)
    trace.fly(settle_s)
    stats = wing_loiter_stats(trace.fly(window_s), entry, h0)
    log(f"loiter about ({entry[0]:.1f}, {entry[1]:.1f}) at {h0:.1f} m: |d-R| mean {stats['mean']:.1f} max "
        f"{stats['max']:.1f} m, alt within {stats['alt']:.2f} m, fitted centre {stats['centre_off']:.1f} m off "
        f"the entry (limit {stats['centre_limit']:.1f}), radius {stats['fitted_r']:.1f} m, mean yaw rate "
        f"{math.degrees(stats['yaw_rate']):+.1f} deg/s")
    return entry, h0, trace, stats


def wing_loiter_failures(stats, mean_limit, max_limit, alt_limit):
    failures = []
    if stats["yaw_rate"] <= 0.0:
        failures.append("loitered the wrong way round")
    if stats["mean"] >= mean_limit or stats["max"] >= max_limit:
        failures.append(f"off the circle: |d-R| mean {stats['mean']:.1f} m, max {stats['max']:.1f} m")
    if stats["alt"] >= alt_limit:
        failures.append(f"height wandered {stats['alt']:.1f} m")
    if stats["centre_off"] >= stats["centre_limit"]:
        failures.append(f"circle centred {stats['centre_off']:.1f} m off the entry point")
    return failures


def scenario_wing_loiter(sitl, rc, fdm):
    """POSHOLD loiters right-hand about the point where it engaged, holding the height."""
    _, _, _, stats = wing_loiter(sitl, rc, fdm)
    assert_no_failures(wing_loiter_failures(stats, 10.0, 20.0, WING_ALT_HOLD_M) + wing_stall_failures(fdm.model))


def scenario_wing_loiter_wind(sitl, rc, fdm):
    """The loiter holds its circle in a 5 m/s wind."""
    _, _, _, stats = wing_loiter(sitl, rc, fdm)
    assert_no_failures(wing_loiter_failures(stats, 15.0, 30.0, 5.0) + wing_stall_failures(fdm.model))


def scenario_wing_loiter_gps_loss(sitl, rc, fdm):
    """In a 5 m/s wind the GPS goes for 30 s: the loiter keeps circling the same way at the same height, drifting
    with the air, and flies back to its circle once the GPS is back."""
    entry, h0, trace, _ = wing_loiter(sitl, rc, fdm, settle_s=30.0, window_s=10.0)
    fdm.gps_valid = False
    dark_s = 30.0
    dark = trace.fly(dark_s)
    fdm.gps_valid = True
    turning = [s.yaw_rate for s in dark]
    alt = max(abs(s.u - h0) for s in dark)
    drift = max(math.hypot(s.e - entry[0], s.n - entry[1]) for s in dark)
    wind = math.hypot(*fdm.model.wind[:2])
    circle_r = WING_LOITER_RADIUS * (WING_CRUISE_AIRSPEED / WING_CRUISE_SPEED) ** 2
    reach = WING_LOITER_RADIUS + 2.0 * circle_r + wind * dark_s
    log(f"{dark_s:.0f} s without GPS: yaw rate {math.degrees(min(turning)):+.1f}..{math.degrees(max(turning)):+.1f} "
        f"deg/s, alt within {alt:.2f} m, furthest {drift:.0f} m from the centre (limit {reach:.0f})")
    failures = []
    if min(turning) <= 0.0:
        failures.append("stopped turning the loiter's way without GPS")
    if alt >= 5.0:
        failures.append(f"height wandered {alt:.1f} m without GPS")
    if drift > reach:
        failures.append(f"flew {drift:.0f} m from the centre without GPS")

    returned = time.monotonic()
    back = None
    while back is None and time.monotonic() - returned < 60.0:
        trace.fly(1.0)
        now = time.monotonic()
        recent = [s for s in trace.samples if s.t >= max(returned, now - 10.0)]
        if now - returned >= 10.0 and all(radius_error(s, entry) < 15.0 for s in recent):
            back = now - 10.0 - returned
    log(f"back on the circle {back:.0f} s after the GPS returned" if back is not None
        else "never got back on the circle")
    if back is None:
        failures.append("did not get back on the circle within 60 s of the GPS returning")
    assert_no_failures(failures + wing_stall_failures(fdm.model))


def scenario_wing_launch_into_poshold(sitl, rc, fdm):
    """Armed with LAUNCH and POSHOLD selected, the launch hands the climb-out to a loiter at the
    handover point, with alt hold alongside."""
    entry, failures = wing_launch_into_hold(sitl, rc, fdm, 9, {BOX_ALTHOLD, BOX_POSHOLD})   # AUX6: POSHOLD
    loiter = WingTrace(sitl, fdm).fly(30.0)
    centre, radius = fit_circle([(s.e, s.n) for s in loiter[-200:]])
    off = math.hypot(centre[0] - entry[0], centre[1] - entry[1])
    log(f"loitering {off:.1f} m from the handover point, radius {radius:.1f} m")
    if off >= 10.0:
        failures.append(f"loiter centred {off:.1f} m from the handover point")
    if abs(radius - WING_LOITER_RADIUS) >= 10.0:
        failures.append(f"loiter radius {radius:.1f} m")
    assert_no_failures(failures + wing_stall_failures(fdm.model))


FP_NAV_TARGETING = 1
FP_NAV_HOLDING = 2
FP_NAV_COMPLETE = 3
FP_NAV_LANDING = 4
WING_MISSION_ALT_M = 40.0
WING_MISSION_HOLD_S = 30.0
WING_MISSION_CLIMB_MPS = 2.0    # alt_hold_climb_rate, the rate a leg climbs at


def wing_waypoint(index, east_m, north_m, kind, duration_ds=0, insert=True, alt_m=WING_MISSION_ALT_M):
    lat = HOME_LAT + north_m / M_PER_DEG
    lon = HOME_LON + east_m / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))
    alt_cm = int((HOME_ALT_M + alt_m) * 100)
    verb = "insert" if insert else "update"
    return f"waypoint {verb} {index} {lat:.7f} {lon:.7f} {alt_cm} 0 {kind} {duration_ds} none"


def wing_mission_cfg(mission, *extra):
    """The wing config with a mission of (east, north, kind, altitude) waypoints."""
    return [*WING_CFG, *extra,
            *[wing_waypoint(i, e, n, kind, insert=i > 0, alt_m=alt) for i, (e, n, kind, alt) in enumerate(mission)]]


# A 400 x 200 m rectangle at 40 m, flown clockwise from the launch heading north: round the far
# corners, over the third, a hold on the way back, and over the start of the long side to finish.
WING_MISSION = [
    (0.0, 300.0, "flyby", 0),
    (400.0, 300.0, "flyby", 0),
    (400.0, 100.0, "flyover", 0),
    (200.0, 100.0, "hold", int(WING_MISSION_HOLD_S * 10)),
    (0.0, 100.0, "flyby", 0),
    (0.0, 300.0, "flyover", 0),
]

WING_MISSION_CFG = [
    *WING_CFG,
    *[wing_waypoint(i, e, n, kind, dur, insert=i > 0) for i, (e, n, kind, dur) in enumerate(WING_MISSION)],
]


def wing_mission_launch(sitl, rc, fdm, armed_at_m=0.0):
    """Arm with LAUNCH and AUTOPILOT selected and throw the airframe: the mission takes the
    climb-out over."""
    rc.set(5, RC_HIGH)                      # AUX2: AUTOPILOT, selected before arming
    wing_throw(sitl, rc, fdm, armed_at_m, not_yet={BOX_AUTOPILOT})
    time.sleep(0.5)
    rc.set(2, WING_CLIMB_STICK)
    wait_for("launch handed to the mission",
             lambda: BOX_LAUNCH not in (m := sitl.modes()) and {BOX_AUTOPILOT, BOX_ALTHOLD, BOX_POSHOLD} <= m,
             timeout=20, interval=0.05)
    wing_launched(fdm)


def bearing_deg(a, b):
    return math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360.0


def scenario_wing_mission(sitl, rc, fdm):
    """The launch hands the climb-out to a mission round a 400 x 200 m rectangle at 40 m. FLYBY corners are
    turned inside their turn distance and outside half of where an arc tangent to both legs at it would pass,
    FLYOVER waypoints are passed over, every straight leg is held to its line once the guidance has closed onto it
    (two look-ahead distances at the leg's groundspeed) until the next turn, the altitude to its climb, the hold
    loiters for its 30 s on its circle, and the finished mission loiters about its last waypoint."""
    wind = fdm.model.wind
    wing_mission_launch(sitl, rc, fdm)
    trace = WingTrace(sitl, fdm, abort_check=True)
    complete_t = trace.fly_until(lambda s: s.nav_state == FP_NAV_COMPLETE, timeout=420, what="mission complete").t
    trace.fly_until(lambda s: s.t - complete_t > 60.0, timeout=70, what="a minute after the mission")
    samples = trace.samples
    points = [(0.0, 0.0)] + [(e, n) for e, n, _, _ in WING_MISSION]
    failures = []

    # each waypoint, against the closest the aircraft came flying the leg to it and just after
    for i, (e, n, kind, _) in enumerate(WING_MISSION):
        leg = [s for s in samples if s.leg == i and s.nav_state == FP_NAV_TARGETING]
        if not leg:
            failures.append(f"never flew the leg to wp{i}")
            continue
        closest = min(math.hypot(s.e - e, s.n - n) for s in samples if leg[0].t <= s.t <= leg[-1].t + 3.0)
        if kind == "flyby":
            prev, here, nxt = points[i], points[i + 1], points[i + 2]
            turn = abs(wrap180(bearing_deg(here, nxt) - bearing_deg(prev, here)))
            d = wing_turn_distance_m(turn, wing_groundspeed(wind, bearing_deg(prev, here)))
            low, high = 0.5 * d * math.tan(math.radians(turn) / 4.0), max(d, WING_ARRIVAL_RADIUS_M) + 10.0
        elif kind == "flyover":
            low, high = 0.0, WING_ARRIVAL_RADIUS_M
        else:
            continue
        log(f"wp{i} {kind}: closest {closest:.1f} m (limits {low:.0f}..{high:.0f})")
        if not low <= closest <= high:
            failures.append(f"wp{i} {kind} passed {closest:.1f} m off (limits {low:.0f}..{high:.0f})")

    # straight legs: the cross-track error from two look-aheads along (past where the launch handed over, on the
    # first) to one short of the end; the legs into and out of the hold run to and from its circle, not along their
    # lines
    for i in range(len(WING_MISSION)):
        if WING_MISSION[i][2] == "hold" or (i > 0 and WING_MISSION[i - 1][2] == "hold"):
            continue
        (ae, an), (be, bn) = points[i], points[i + 1]
        length = math.hypot(be - ae, bn - an)
        ue, un = (be - ae) / length, (bn - an) / length
        l1 = wing_l1_m(wing_groundspeed(wind, bearing_deg(points[i], points[i + 1])))
        start = 2.0 * l1 + (max(0.0, (samples[0].e - ae) * ue + (samples[0].n - an) * un) if i == 0 else 0.0)
        xtrack = []
        for s in samples:
            if s.leg == i and s.nav_state == FP_NAV_TARGETING:
                along = (s.e - ae) * ue + (s.n - an) * un
                if start <= along <= length - l1:
                    xtrack.append(abs((s.e - ae) * un - (s.n - an) * ue))
        what = f"on the line of the leg to wp{i}"
        leg_rms = rms(xtrack, what)
        log(f"leg to wp{i}: cross-track rms {leg_rms:.2f} m, max {max(xtrack):.2f} m over {len(xtrack)} samples "
            f"from {start:.0f} to {length - l1:.0f} m along")
        if leg_rms > 5.0 or max(xtrack) > 15.0:
            failures.append(f"leg to wp{i}: cross-track rms {leg_rms:.1f} m (limit 5), max {max(xtrack):.1f} m "
                            f"(limit 15)")

    # altitude, against a climb at the leg rate from where the mission took over
    t0, h0 = samples[0].t, samples[0].u
    alt_errs = []
    for s in samples:
        ramp = min(WING_MISSION_ALT_M, h0 + WING_MISSION_CLIMB_MPS * (s.t - t0)) if h0 < WING_MISSION_ALT_M \
            else max(WING_MISSION_ALT_M, h0 - WING_MISSION_CLIMB_MPS * (s.t - t0))
        alt_errs.append(abs(s.u - ramp))
    log(f"altitude: from {h0:.1f} m, within {max(alt_errs):.2f} m of the climb")
    if max(alt_errs) > WING_ALT_HOLD_M:
        failures.append(f"altitude {max(alt_errs):.1f} m off the climb")

    # the hold: on its circle, for its duration
    hold_index = next(i for i, wp in enumerate(WING_MISSION) if wp[2] == "hold")
    holding = [s for s in samples if s.leg == hold_index and s.nav_state == FP_NAV_HOLDING]
    if holding:
        held_s = holding[-1].t - holding[0].t
        hold_err = mean([radius_error(s, points[hold_index + 1]) for s in holding], "holding")
        log(f"hold: {held_s:.1f} s, mean |d-R| {hold_err:.1f} m")
        if abs(held_s - WING_MISSION_HOLD_S) > 3.0 or hold_err > 10.0:
            failures.append(f"hold {held_s:.1f} s with mean |d-R| {hold_err:.1f} m")
    else:
        failures.append("never held")

    # finished: loitering about the last waypoint
    loiter_err = mean([radius_error(s, points[-1]) for s in samples if s.t - complete_t > 20.0], "after the mission")
    log(f"complete: loitering about the last waypoint, mean |d-R| {loiter_err:.1f} m")
    if loiter_err > 15.0:
        failures.append(f"loiter after the mission {loiter_err:.1f} m off its circle")
    if BOX_ARM not in sitl.modes():
        failures.append("disarmed")
    assert_no_failures(failures + wing_stall_failures(fdm.model))


# --- Wing landing -------------------------------------------------------
# A mission out 300 m north at 50 m to a LAND waypoint at home whose altitude is the ground's: the
# wing loiters down over home to the 30 m approach altitude, flies a left hand pattern round to a
# 150 m final, and lands south, the course it flew in on. Every landing setting is the firmware's default.

LANDING_PHASE_NAMES = ["idle", "loiter down", "align", "downwind", "base", "final", "flare", "touchdown", "go around"]
(LANDING_IDLE, LANDING_LOITER_DOWN, LANDING_ALIGN, LANDING_DOWNWIND, LANDING_BASE, LANDING_FINAL,
 LANDING_FLARE) = range(7)
LANDING_GO_AROUND = 8
LANDING_PATTERN = [LANDING_LOITER_DOWN, LANDING_ALIGN, LANDING_DOWNWIND, LANDING_BASE, LANDING_FINAL, LANDING_FLARE]
GO_AROUND_STICK = 1
GO_AROUND_SLOPE = 3

WING_AUTOLAND_CFG = [
    *WING_CFG,
    *WING_MOTOR_STOP_CFG,
    "set debug_mode = AUTOPILOT_LANDING",
    wing_waypoint(0, 0.0, 300.0, "flyover", insert=False, alt_m=50.0),
    wing_waypoint(1, 0.0, 0.0, "land", alt_m=0.0),
]


def phases(samples):
    seen = []
    for s in samples:
        if s.phase != LANDING_IDLE and (not seen or seen[-1] != s.phase):
            seen.append(s.phase)
    return seen


def wing_fly_to_contact(trace, timeout, each=None):
    """Fly to the first ground contact; a crash there ends the flight too, for wing_landing_failures to report."""
    model = trace.fdm.model
    try:
        trace.fly_until(lambda s: model.contact is not None, timeout=timeout, what="ground contact", each=each)
    except AssertionError:
        if model.contact is None:
            raise


def wing_contact_failures(model, heading_deg=None, touchdown=None, roll_deg=WING_CONTACT_ROLL_DEG):
    """The first contact: sinking slowly, wings level, nose up, flying and slow; with a landing heading, the ground
    track on it; with a touchdown point too, down near it along and across that heading."""
    c = model.contact
    track = math.degrees(math.atan2(c["ve"], c["vn"])) % 360.0
    log(f"contact: track {track:.0f} deg, {-c['vz']:.2f} m/s down, {c['gs']:.1f} m/s over the ground "
        f"({c['airspeed']:.1f} airspeed), bank {c['roll']:+.1f} deg, pitch {c['pitch']:+.1f} deg"
        f"{', stalled' if c['stalled'] else ''}")
    failures = []
    if heading_deg is not None:
        if abs(wrap180(track - heading_deg)) > WING_TRACK_TOLERANCE_DEG:
            failures.append(f"landed on a {track:.0f} deg track, not {heading_deg:.0f} deg")
        if touchdown is not None:
            ue, un = math.sin(math.radians(heading_deg)), math.cos(math.radians(heading_deg))
            de, dn = c["e"] - touchdown[0], c["n"] - touchdown[1]
            along_m, xtrack_m = de * ue + dn * un, de * un - dn * ue
            log(f"contact {along_m:+.1f} m along and {xtrack_m:+.1f} m across {heading_deg:.0f} deg from the "
                f"touchdown point")
            lo, hi = WING_TOUCHDOWN_ALONG_M
            if not lo <= along_m <= hi:
                failures.append(f"came down {along_m:+.1f} m along (limits {lo:+.0f}..{hi:+.0f})")
            if abs(xtrack_m) > WING_TOUCHDOWN_ACROSS_M:
                failures.append(f"came down {xtrack_m:+.1f} m across (limit {WING_TOUCHDOWN_ACROSS_M:.0f})")
    if -c["vz"] > WING_CONTACT_SINK:
        failures.append(f"came down at {-c['vz']:.1f} m/s")
    if abs(c["roll"]) > roll_deg:
        failures.append(f"came down banked {c['roll']:+.1f} deg")
    if not WING_CONTACT_PITCH_DEG[0] <= c["pitch"] <= WING_CONTACT_PITCH_DEG[1]:
        failures.append(f"came down at {c['pitch']:+.1f} deg of pitch")
    if c["stalled"]:
        failures.append("came down stalled")
    if c["airspeed"] > WING_CONTACT_AIRSPEED:
        failures.append(f"came down at {c['airspeed']:.1f} m/s airspeed (limit {WING_CONTACT_AIRSPEED:.1f})")
    return failures


def wing_ground_failures(sitl, fdm, motor_stop):
    """After the contact, the motor drops to idle (to stopped with MOTOR_STOP) and the ground run comes to a
    stop and the landing disarms soon after."""
    m = fdm.model
    stop_t = disarm_t = None
    motor = 0.0
    powered_s = 0.0
    t0 = time.monotonic()
    while time.monotonic() - t0 < 30.0 and disarm_t is None:
        now = time.monotonic()
        assert not m.crashed, f"crashed on the ground: {m.crash_reason}"
        if stop_t is None and math.hypot(m.vel[0], m.vel[1]) < 0.5:
            stop_t = now
        if BOX_ARM not in sitl.modes():
            disarm_t = now
        else:
            motor = max(motor, fdm.motors.motors[0])
            if fdm.motors.motors[0] > WING_MOTOR_IDLE:
                powered_s = now - t0
        time.sleep(0.05)
    if disarm_t is None:
        return ["still armed 30 s after the contact"]
    stop_t = stop_t if stop_t is not None else disarm_t
    slid = math.hypot(m.pos[0] - m.contact["e"], m.pos[1] - m.contact["n"])
    log(f"stopped {stop_t - t0:.1f} s and disarmed {disarm_t - t0:.1f} s after the contact; slid {slid:.0f} m; "
        f"motor at most {motor:.3f} on the ground, above idle for {powered_s:.1f} s")
    failures = []
    if disarm_t - stop_t > WING_STOP_TO_DISARM_S:
        failures.append(f"disarmed {disarm_t - stop_t:.1f} s after it stopped")
    if powered_s > WING_MOTOR_CUT_S:
        failures.append(f"motor above idle for {powered_s:.1f} s after the contact")
    if motor_stop and motor > 0.0:
        failures.append(f"motor ran at {motor:.3f} on the ground with MOTOR_STOP")
    return failures


def wing_landing_failures(sitl, fdm, heading_deg=None, touchdown=None, motor_stop=True, roll_deg=WING_CONTACT_ROLL_DEG):
    failures = wing_contact_failures(fdm.model, heading_deg, touchdown, roll_deg) + wing_stall_failures(fdm.model)
    time.sleep(0.1)
    if fdm.model.crashed:
        return [f"crashed: {fdm.model.crash_reason}"] + failures
    return failures + wing_ground_failures(sitl, fdm, motor_stop)


def wing_autoland(sitl, rc, fdm, heading_deg=180.0, touchdown=(0.0, 0.0), each=None, timeout=420):
    """Launch into the mission and fly it to the ground; returns (trace, failures)."""
    wing_mission_launch(sitl, rc, fdm)
    trace = WingTrace(sitl, fdm, debug=True)
    try:
        wing_fly_to_contact(trace, timeout, each)
    finally:
        log(f"phases: {', '.join(LANDING_PHASE_NAMES[p] for p in phases(trace.samples))}; go-arounds "
            f"{trace.samples[-1].attempts if trace.samples else 0}, last cause "
            f"{trace.samples[-1].cause if trace.samples else 0}")
    return trace, wing_landing_failures(sitl, fdm, heading_deg, touchdown)


def scenario_wing_autoland(sitl, rc, fdm):
    """Launch, fly out and back to the LAND waypoint at home, and land on the approach: every phase in order, the
    contact on the course flown in, near the touchdown point, slow, level, nose up and flying, the motor stopped
    on the ground, and disarmed soon after the ground run stops."""
    trace, failures = wing_autoland(sitl, rc, fdm)
    seen = phases(trace.samples)
    if seen[:len(LANDING_PATTERN)] != LANDING_PATTERN:
        failures.append(f"phases {[LANDING_PHASE_NAMES[p] for p in seen]}")
    assert_no_failures(failures)


def scenario_wing_autoland_crosswind(sitl, rc, fdm):
    """The landing in a 5 m/s wind from the west, straight across the final."""
    scenario_wing_autoland(sitl, rc, fdm)


# In to the LAND waypoint at home from 300 m south of it: the course flown in lands north.
WING_AUTOLAND_NORTH_CFG = [
    *WING_CFG,
    *WING_MOTOR_STOP_CFG,
    "set debug_mode = AUTOPILOT_LANDING",
    wing_waypoint(0, 0.0, -300.0, "flyover", insert=False, alt_m=50.0),
    wing_waypoint(1, 0.0, 0.0, "land", alt_m=0.0),
]
WING_LAND_MAX_TAILWIND = 3.0      # m/s, ap_wing_land_max_tailwind


def wing_autoland_north(sitl, rc, fdm):
    """Land at home and require the landing to have been flown north, through every phase."""
    trace, failures = wing_autoland(sitl, rc, fdm, heading_deg=0.0)
    seen = phases(trace.samples)
    if seen[:len(LANDING_PATTERN)] != LANDING_PATTERN:
        failures.append(f"phases {[LANDING_PHASE_NAMES[p] for p in seen]}")
    assert_no_failures(failures)


def scenario_wing_autoland_headwind(sitl, rc, fdm):
    """In from the south, the landing north into an 8 m/s wind straight down the final."""
    wing_autoland_north(sitl, rc, fdm)


def scenario_wing_autoland_tailwind_flip(sitl, rc, fdm):
    """In from the north the course flown in would land south with a 5 m/s tailwind, more than
    ap_wing_land_max_tailwind: the landing turns round and lands north, into the wind."""
    assert -fdm.model.wind[1] > WING_LAND_MAX_TAILWIND
    wing_autoland_north(sitl, rc, fdm)


WING_GO_AROUND_HEIGHT_M = 12.0
WING_GO_AROUND_MIN_HEIGHT_M = 10.0
WING_LOITER_SETTLE_S = 15.0


def wing_go_around_failures(trace, start, cause):
    """From the start sample the landing goes around: it climbs, wings level from the time a roll out of the maximum
    bank takes until 3 s in, settles back to the approach altitude in the loiter, and comes down the pattern again."""
    if start is None:
        return [f"never down to {WING_GO_AROUND_HEIGHT_M:.0f} m on final"]
    failures = []
    if start.u < WING_GO_AROUND_MIN_HEIGHT_M:
        failures.append(f"final reached {WING_GO_AROUND_HEIGHT_M:.0f} m only at {start.u:.1f} m")
    after = [s for s in trace.samples if s.t >= start.t]
    around = next((s for s in after if s.phase == LANDING_GO_AROUND), None)
    if around is None:
        return failures + ["never went around"]
    log(f"went around {around.t - start.t:.1f} s after the cause, from {around.u:.1f} m, for cause {around.cause}")
    if around.t - start.t > 2.0:
        failures.append(f"went around {around.t - start.t:.1f} s after the cause")
    if around.cause != cause:
        failures.append(f"went around for cause {around.cause}, not {cause}")
    climbing = next((s for s in after if s.vu > 1.0), None)
    if climbing is None or climbing.t - around.t > 2.0:
        failures.append("did not climb at over 1 m/s within 2 s")
    else:
        log(f"climbing at over 1 m/s {climbing.t - around.t:.1f} s after going around")
    roll = max(abs(s.roll) for s in after if around.t + wing_roll_out_s() <= s.t <= around.t + 3.0)
    log(f"bank within {roll:.1f} deg of level from {wing_roll_out_s():.2f} s to 3 s after going around")
    if roll > 10.0:
        failures.append(f"banked {roll:.1f} deg going around")
    back = next((i for i, s in enumerate(after) if s.t > around.t and s.phase == LANDING_LOITER_DOWN), None)
    if back is None:
        return failures + ["never came back to the loiter"]
    loiter = list(itertools.takewhile(lambda s: s.phase == LANDING_LOITER_DOWN, after[back:]))
    settled = [s for s in loiter if s.t - loiter[0].t >= WING_LOITER_SETTLE_S]
    log(f"back in the loiter at {loiter[0].u:.1f} m, {loiter[0].t - around.t:.0f} s after going around, for "
        f"{loiter[-1].t - loiter[0].t:.0f} s")
    if not settled:
        return failures + [f"left the loiter within {WING_LOITER_SETTLE_S:.0f} s"]
    alt_err = max(abs(s.u - WING_APPROACH_ALT_M) for s in settled)
    log(f"from {WING_LOITER_SETTLE_S:.0f} s in the loiter, within {alt_err:.1f} m of the approach altitude")
    if alt_err > WING_ALT_HOLD_M:
        failures.append(f"loitered {alt_err:.1f} m off the approach altitude")
    return failures


def on_final(s):
    return s.phase == LANDING_FINAL and s.u <= WING_GO_AROUND_HEIGHT_M


def scenario_wing_go_around_stick(sitl, rc, fdm):
    """The pilot pushes the pitch stick at 12 m on final: the wing goes around and lands next time."""
    state = {}

    def push(s):
        if "start" not in state and on_final(s):
            state["start"] = s
            log(f"stick at {s.u:.1f} m on final")
            rc.set(1, 1000)
        elif "start" in state and s.t - state["start"].t > 0.5:
            rc.set(1, RC_MID)

    trace, failures = wing_autoland(sitl, rc, fdm, each=push, timeout=600)
    failures += wing_go_around_failures(trace, state.get("start"), GO_AROUND_STICK)
    if trace.samples[-1].attempts != 1:
        failures.append(f"{trace.samples[-1].attempts} go-arounds")
    assert_no_failures(failures)


def scenario_wing_go_around_baro(sitl, rc, fdm):
    """The altitude jumps 15 m on final, baro and GPS together, more than the 10 m default slope tolerance: the wing
    goes around, the error clears on the climb, and it lands next time."""
    state = {}

    def bias(s):
        if "start" not in state and on_final(s):
            state["start"] = s
            log(f"altitude 15 m high at {s.u:.1f} m on final")
            fdm.altitude_bias_m = 15.0
        elif "start" in state and "around" not in state and s.phase == LANDING_GO_AROUND:
            state["around"] = s.t
        elif "around" in state and fdm.altitude_bias_m and s.t - state["around"] > 2.0:
            log(f"altitude right again at {s.u:.1f} m, climbing out")
            fdm.altitude_bias_m = 0.0

    trace, failures = wing_autoland(sitl, rc, fdm, each=bias, timeout=600)
    assert_no_failures(failures + wing_go_around_failures(trace, state.get("start"), GO_AROUND_SLOPE))


def scenario_wing_go_around_gps(sitl, rc, fdm):
    """The GPS goes for 20 s on the downwind leg: the wing circles at the approach altitude until it is
    back, then flies the pattern again from the loiter and lands."""
    state = {"alts": []}

    def outage(s):
        if "t" not in state and s.phase == LANDING_DOWNWIND:
            state["t"] = s.t
            log(f"GPS dark on the downwind leg at {s.u:.1f} m")
            fdm.gps_valid = False
        elif "t" in state and not fdm.gps_valid:
            state["alts"].append(s.u)
            if s.t - state["t"] > 20.0:
                log("GPS back")
                fdm.gps_valid = True

    trace, failures = wing_autoland(sitl, rc, fdm, each=outage, timeout=600)
    alts = require(state["alts"], "without the GPS")
    log(f"circled between {min(alts):.1f} and {max(alts):.1f} m without the GPS")
    if max(abs(a - WING_APPROACH_ALT_M) for a in alts) > 5.0:
        failures.append(f"held {min(alts):.1f}..{max(alts):.1f} m without the GPS")
    if LANDING_GO_AROUND not in phases(trace.samples):
        failures.append("did not go round again for the lost position")
    assert_no_failures(failures)


# A rectangle out of home to a LAND waypoint 300 m east, its altitude the ground's: it lands south,
# the course it flies in on.
WING_MISSION_LAND = [
    (0.0, 300.0, "flyby", 40.0),
    (300.0, 300.0, "flyover", 40.0),
    (300.0, 0.0, "land", 0.0),
]
WING_MISSION_LAND_CFG = wing_mission_cfg(WING_MISSION_LAND, *WING_MOTOR_STOP_CFG, "set debug_mode = AUTOPILOT_LANDING")


def scenario_wing_mission_land(sitl, rc, fdm):
    """A mission ending at a LAND waypoint away from home lands there, on the course flown in to it."""
    _, failures = wing_autoland(sitl, rc, fdm, touchdown=WING_MISSION_LAND[-1][:2])
    assert_no_failures(failures)


# --- Wing rescue ---------------------------------------------------------
# The wing flies a mission out to 400 m at 30 m, and the rescue takes over from it: a switch, or an
# rx loss with the GPS-RESCUE procedure. It climbs to 50 m in a loiter it enters where it is, leaves
# it once at height and heading home, flies home on the line from the loiter's centre, loiters down
# about home and lands on the approach along the course it was launched on, north but for the wind's drift.

WING_RESCUE_ALT_M = 50.0
WING_RESCUE_OUT_M = 400.0
WING_RESCUE_CFG = [
    *WING_CFG,
    *WING_MOTOR_STOP_CFG,
    "aux 4 46 4 1700 2100 0 0",   # GPS RESCUE on AUX5
    "set gps_rescue_alt_mode = FIXED_ALT",
    f"set gps_rescue_return_alt = {int(WING_RESCUE_ALT_M)}",
    wing_waypoint(0, 0.0, 450.0, "flyover", insert=False, alt_m=30.0),
]


def wing_fly_out(sitl, rc, fdm, armed_at_m=0.0):
    """Launch into the mission and fly it out to the rescue's starting point."""
    wing_mission_launch(sitl, rc, fdm, armed_at_m)
    trace = WingTrace(sitl, fdm, abort_check=True)
    s = trace.fly_until(lambda s: math.hypot(s.e, s.n) > WING_RESCUE_OUT_M, timeout=120, what="the rescue's start")
    log(f"rescue from {math.hypot(s.e, s.n):.0f} m out at {s.u:.1f} m")
    trace.abort_check = False
    return trace


def wing_rescue_start(fdm):
    """Where the rescue starts, and the centre of the loiter it climbs in: a right hand circle
    through the aircraft, on its course."""
    m = fdm.model
    e, n = m.pos[0], m.pos[1]
    ve, vn = m.vel[0], m.vel[1]
    v = math.hypot(ve, vn)
    return time.monotonic(), (e + WING_LOITER_RADIUS * vn / v, n - WING_LOITER_RADIUS * ve / v)


def wing_rescue_home(trace, start):
    """Fly the rescue that started at start home and land, and check each part of it; returns the
    failures."""
    t0, centre = start
    trace.fly_until(lambda s: s.t > t0 + 2.0 and s.leg == 2 and s.nav_state == FP_NAV_LANDING, timeout=240,
                    what="the rescue's landing")
    wing_fly_to_contact(trace, timeout=300)
    samples = [s for s in trace.samples if s.t >= t0]
    failures = []

    # the climb: a loiter entered where the rescue began, until at height and heading for home
    climb = require([s for s in samples if s.leg == 0], "climbing")
    errs = [radius_error(s, centre) for s in climb]
    log(f"climbing loiter from {climb[0].u:.1f} m: {climb[-1].t - climb[0].t:.1f} s, mean |d-R| "
        f"{mean(errs, 'climbing'):.1f} m, max {max(errs):.1f} m")
    if mean(errs, "climbing") > 15.0:
        failures.append(f"climbing loiter mean |d-R| {mean(errs, 'climbing'):.1f} m")

    # home: on the line from the loiter's centre, at the return altitude
    leg = require([s for s in samples if s.leg == 1], "flying home")
    length = math.hypot(*centre)
    ue, un = -centre[0] / length, -centre[1] / length
    l1 = wing_l1_m(WING_CRUISE_AIRSPEED)
    on_line = [s for s in leg if (s.e - centre[0]) * ue + (s.n - centre[1]) * un >= 60.0
               and math.hypot(s.e, s.n) > WING_LOITER_RADIUS + l1]
    xtrack = [abs((s.e - centre[0]) * un - (s.n - centre[1]) * ue) for s in on_line]
    line_rms = rms(xtrack, "on the home line")
    alt_err = max(abs(s.u - WING_RESCUE_ALT_M) for s in on_line)
    log(f"left the loiter at {leg[0].u:.1f} m, {length:.0f} m from home; home line cross-track rms {line_rms:.2f} m, "
        f"max {max(xtrack):.2f} m over {len(xtrack)} samples; altitude within {alt_err:.2f} m of the return "
        f"altitude on the line, {max(abs(s.u - WING_RESCUE_ALT_M) for s in leg):.2f} m turning onto it")
    if line_rms > 6.0:
        failures.append(f"home line cross-track rms {line_rms:.1f} m (limit 6)")
    if alt_err > WING_ALT_HOLD_M:
        failures.append(f"return altitude {alt_err:.1f} m off")

    # the loiter about home takes over as it is turned onto, and the landing from it
    land = require([s for s in samples if s.leg == 2], "landing")
    capture = math.hypot(land[0].e, land[0].n)
    log(f"home loiter took over {capture:.0f} m from home (limit {WING_LOITER_RADIUS + l1 + 10.0:.0f})")
    if capture > WING_LOITER_RADIUS + l1 + 10.0:
        failures.append(f"home loiter took over {capture:.0f} m out")
    return failures


def scenario_wing_rescue(sitl, rc, fdm, armed_at_m=0.0):
    """The pilot flips the rescue switch 400 m out at 30 m."""
    trace = wing_fly_out(sitl, rc, fdm, armed_at_m)
    start = wing_rescue_start(fdm)
    rc.set(8, RC_HIGH)      # AUX5: GPS RESCUE
    wait_for("the rescue flying (AUTOPILOT, no FAILSAFE)",
             lambda: BOX_AUTOPILOT in (m := sitl.modes()) and BOX_FAILSAFE not in m, timeout=5)
    failures = wing_rescue_home(trace, start)
    assert_no_failures(failures + wing_landing_failures(sitl, fdm, fdm.launch_course_deg, (0.0, 0.0)))


def scenario_wing_rescue_wind(sitl, rc, fdm):
    """The rescue in a 5 m/s wind from the west, across the way home."""
    scenario_wing_rescue(sitl, rc, fdm)


def scenario_wing_rescue_armed_in_hand(sitl, rc, fdm):
    """The rescue from a wing armed in the thrower's hand 1.5 m up: its landing at home takes the
    ground to be ap_wing_land_launch_height below where it was armed, and comes down to the autoland's
    limits rather than stopping its motor and stalling a metre and a half up."""
    scenario_wing_rescue(sitl, rc, fdm, armed_at_m=1.5)


def scenario_wing_rx_loss_rescue(sitl, rc, fdm):
    """The RC link goes 400 m out with the GPS-RESCUE procedure: the failsafe flies the rescue."""
    trace = wing_fly_out(sitl, rc, fdm)
    t0 = time.monotonic()
    rc.stop_stream()
    wait_for("failsafe flying the rescue (FAILSAFE + AUTOPILOT)",
             lambda: {BOX_FAILSAFE, BOX_AUTOPILOT} <= sitl.modes(), timeout=5, interval=0.05)
    engaged_s = time.monotonic() - t0
    log(f"rescue engaged {engaged_s:.2f} s after the link went (failsafe_delay {WING_FAILSAFE_DELAY_S:.1f} s)")
    failures = wing_rescue_home(trace, wing_rescue_start(fdm))
    if engaged_s > WING_FAILSAFE_DELAY_S + 0.5:
        failures.append(f"engaged {engaged_s:.2f} s after the link went")
    assert_no_failures(failures + wing_landing_failures(sitl, fdm, fdm.launch_course_deg, (0.0, 0.0)))


# A mission out and back that ends at a FLYOVER: a wing cannot come down where it ends.
WING_RX_CONTINUE_MISSION = [
    (0.0, 300.0, "flyby", 30.0),
    (300.0, 300.0, "flyby", 30.0),
    (300.0, 0.0, "flyover", 30.0),
]
WING_RX_CONTINUE_CFG = wing_mission_cfg(WING_RX_CONTINUE_MISSION, *WING_MOTOR_STOP_CFG,
                                        "set ap_rx_loss_policy = CONTINUE")


def scenario_wing_rx_loss_continue(sitl, rc, fdm):
    """The RC link goes on the mission's first leg with the rx loss policy CONTINUE: the mission flies
    on through the failsafe to its end, then the rescue flies home and lands there."""
    wing_mission_launch(sitl, rc, fdm)
    trace = WingTrace(sitl, fdm, abort_check=True)
    trace.fly_until(lambda s: s.n > 150.0, timeout=60, what="150 m out on the first leg")
    rc.stop_stream()
    wait_for("failsafe with the mission flying on (FAILSAFE + AUTOPILOT)",
             lambda: {BOX_FAILSAFE, BOX_AUTOPILOT} <= sitl.modes(), timeout=5, interval=0.05)
    last = len(WING_RX_CONTINUE_MISSION) - 1
    trace.fly_until(lambda s: s.leg == last, timeout=180, what="the mission's last leg")
    # the mission's end hands straight over to the rescue, whose plan starts again from its first leg
    took_over = trace.fly_until(lambda s: s.leg < last, timeout=120, what="the rescue")
    legs = sorted({s.leg for s in trace.samples[:-1]})
    log(f"flew legs {legs} through the failsafe; the rescue took over {math.hypot(took_over.e, took_over.n):.0f} m "
        f"from home")
    trace.abort_check = False
    wing_fly_to_contact(trace, timeout=400)
    failures = [] if legs == list(range(len(WING_RX_CONTINUE_MISSION))) else [f"legs flown {legs}"]
    assert_no_failures(failures + wing_landing_failures(sitl, fdm, fdm.launch_course_deg, (0.0, 0.0)))


def wing_roll_out_s():
    """How long a roll out of the maximum bank takes."""
    return WING_MAX_BANK_DEG / WING_ROLL_SLEW_DPS + WING_ROLL_LAG_S


def wing_descent_failures(samples, roll_deg=WING_CONTACT_ROLL_DEG):
    """Circling down, then banked no more than roll_deg from the time a roll out of the maximum bank takes after
    descending through the level height."""
    high = require([s for s in samples if 15.0 < s.u < 25.0], "between 15 and 25 m")
    below = require([s for s in samples if s.u < WING_LEVEL_HEIGHT_M], f"below {WING_LEVEL_HEIGHT_M:.0f} m")
    roll_out_s = wing_roll_out_s()
    level = [s for s in below if s.t >= below[0].t + roll_out_s]
    log(f"descending: bank {min(s.roll for s in high):+.1f}..{max(s.roll for s in high):+.1f} deg between 15 and "
        f"25 m, |bank| at most {max((abs(s.roll) for s in level), default=0.0):.1f} deg from {roll_out_s:.2f} s "
        f"after {WING_LEVEL_HEIGHT_M:.0f} m")
    failures = []
    if max(abs(s.roll) for s in high) < 10.0:
        failures.append("did not circle on the way down")
    if level and max(abs(s.roll) for s in level) >= roll_deg:
        failures.append(f"banked {max(abs(s.roll) for s in level):.1f} deg below {WING_LEVEL_HEIGHT_M:.0f} m")
    return failures


WING_RX_LAND_CFG = [
    *WING_CFG,
    "set failsafe_landing_time = 45",   # 45 s, to tell a disarm on the ground from the landing timeout
    wing_waypoint(0, 0.0, 450.0, "flyover", insert=False, alt_m=30.0),
]


def scenario_wing_rx_loss_land(sitl, rc, fdm):
    """The RC link goes with the AUTO-LAND procedure out of sight of home: not knowing the ground there, the wing
    circles down about where it is, more widely near home's ground, comes down slowly and nose up under power, cuts
    the motor on contact, and disarms once it is still on the ground, long before the failsafe landing time would."""
    wing_mission_launch(sitl, rc, fdm)
    trace = WingTrace(sitl, fdm, abort_check=True)
    trace.fly_until(lambda s: s.n > 250.0 and s.u > 28.0, timeout=90, what="250 m out at 28 m")
    trace.abort_check = False
    t0 = time.monotonic()
    rc.stop_stream()
    wait_for("failsafe landing (FAILSAFE, ALTHOLD and POSHOLD, no AUTOPILOT)",
             lambda: {BOX_FAILSAFE, BOX_ALTHOLD, BOX_POSHOLD} <= (m := sitl.modes()) and BOX_AUTOPILOT not in m,
             timeout=5, interval=0.05)
    start = len(trace.samples)
    wing_fly_to_contact(trace, timeout=60)
    samples = trace.samples[start:]
    failures = wing_descent_failures(samples, WING_UNKNOWN_GROUND_CONTACT_ROLL_DEG)
    drift = max(math.hypot(s.e - samples[0].e, s.n - samples[0].n) for s in samples)
    reach = 2.0 * max(WING_LOITER_RADIUS, wing_circle_m(WING_UNKNOWN_GROUND_LOW_BANK_DEG))
    log(f"came down {drift:.0f} m at most from where the failsafe engaged (limit {reach:.0f})")
    if drift > reach:
        failures.append(f"wandered {drift:.0f} m")
    failures += wing_landing_failures(sitl, fdm, motor_stop=False, roll_deg=WING_UNKNOWN_GROUND_CONTACT_ROLL_DEG)
    disarmed_s = time.monotonic() - t0
    log(f"disarmed {disarmed_s:.1f} s after the link went (failsafe_landing_time 45 s)")
    if disarmed_s > WING_FAILSAFE_DELAY_S + 45.0 - 5.0:
        failures.append(f"disarmed {disarmed_s:.1f} s after the link went, by the failsafe landing time")
    assert_no_failures(failures)


def scenario_wing_rescue_gps_loss(sitl, rc, fdm):
    """The GPS goes on the way home from a switched rescue. The rescue circles where it is at its
    height for 30 s, then gives up, and the switch's descent circles down without a position, not knowing where
    the ground is: more widely near home's ground, slowly and nose up under power, it cuts the motor on the
    contact, and disarms once still."""
    trace = wing_fly_out(sitl, rc, fdm)
    rc.set(8, RC_HIGH)      # AUX5: GPS RESCUE
    rc.set(5, RC_LOW)       # AUX2: AUTOPILOT off, so the switch alone flies the rescue
    lost = trace.fly_until(lambda s: s.leg == 1 and math.hypot(s.e, s.n) < WING_RESCUE_OUT_M - 50.0, timeout=120,
                           what="the way home")
    fdm.gps_valid = False
    log(f"GPS dark {math.hypot(lost.e, lost.n):.0f} m out at {lost.u:.1f} m")

    # the plan's abort ends the switch's AUTOPILOT request at once, so it shows as the mode dropping
    ride_out = []
    while BOX_AUTOPILOT in sitl.modes():
        assert time.monotonic() - lost.t < 45.0, "the rescue never gave up without a position"
        ride_out.append(trace.sample())
        time.sleep(0.1)
    gave_up_s = time.monotonic() - lost.t
    drift = max(math.hypot(s.e - lost.e, s.n - lost.n) for s in require(ride_out, "riding it out"))
    alt = max(abs(s.u - lost.u) for s in ride_out)
    # straight on until the position goes, then the no-position circle at its bank for the plant's airspeed
    circle_r = WING_LOITER_RADIUS * (WING_CRUISE_AIRSPEED / WING_CRUISE_SPEED) ** 2
    reach = WING_CRUISE_AIRSPEED * WING_GPS_LOST_S + 2.0 * circle_r
    log(f"rode it out for {gave_up_s:.1f} s within {drift:.0f} m of where the GPS went (limit {reach:.0f}), height "
        f"within {alt:.1f} m")
    failures = []
    if not 30.0 <= gave_up_s <= 37.0:
        failures.append(f"gave up {gave_up_s:.1f} s after the GPS went")
    if drift > reach or alt > 5.0:
        failures.append(f"ride-out wandered {drift:.0f} m, {alt:.1f} m in height")

    assert {BOX_ALTHOLD, BOX_POSHOLD} <= sitl.modes(), "the switch's descent is not in alt hold and position hold"
    start = len(trace.samples)
    wing_fly_to_contact(trace, timeout=90)
    samples = trace.samples[start:]
    failures += wing_descent_failures(samples, WING_UNKNOWN_GROUND_CONTACT_ROLL_DEG)
    circling = [s for s in samples if s.u > WING_LEVEL_HEIGHT_M]
    drift = max(math.hypot(s.e - lost.e, s.n - lost.n) for s in require(circling, "circling down"))
    reach = WING_CRUISE_AIRSPEED * WING_GPS_LOST_S + 2.0 * wing_circle_m(WING_UNKNOWN_GROUND_BANK_DEG)
    log(f"circled down to {WING_LEVEL_HEIGHT_M:.0f} m within {drift:.0f} m of where the GPS went (limit {reach:.0f})")
    if drift > reach:
        failures.append(f"descent wandered {drift:.0f} m")
    if BOX_FAILSAFE in sitl.modes():
        failures.append("entered failsafe")
    assert_no_failures(failures + wing_landing_failures(sitl, fdm, motor_stop=False,
                                                        roll_deg=WING_UNKNOWN_GROUND_CONTACT_ROLL_DEG))


def ground_north(height_m, start_m, end_m, beyond_slope=0.0):
    """Level ground at home's height out to start_m north of home, sloping to height_m at end_m, and rising
    beyond_slope a metre north of there."""
    def ground(e, n):
        if n <= start_m:
            return 0.0
        if n <= end_m:
            return height_m * (n - start_m) / (end_m - start_m)
        return height_m + (n - end_m) * beyond_slope
    return ground


def wing_descent_cfg(north_m, alt_m, *extra):
    """A flight out north at alt_m to north_m with the AUTO-LAND failsafe procedure and MOTOR_STOP."""
    return [*WING_CFG, *WING_MOTOR_STOP_CFG, *extra,
            "set failsafe_landing_time = 250",
            wing_waypoint(0, 0.0, north_m, "flyover", insert=False, alt_m=alt_m)]


def wing_descent_fly(sitl, rc, fdm, out_m, alt_m):
    """Launch, fly out to out_m north at alt_m, and lose the link there; flies the failsafe landing to the first
    contact, returning the trace's samples from the failsafe and the motor output alongside each."""
    wing_mission_launch(sitl, rc, fdm)
    trace = WingTrace(sitl, fdm, abort_check=True)
    trace.fly_until(lambda s: s.n > out_m and s.u > alt_m - 2.0, timeout=200, what=f"{out_m:.0f} m out")
    trace.abort_check = False
    rc.stop_stream()
    wait_for("failsafe landing (FAILSAFE, ALTHOLD and POSHOLD, no AUTOPILOT)",
             lambda: {BOX_FAILSAFE, BOX_ALTHOLD, BOX_POSHOLD} <= (m := sitl.modes()) and BOX_AUTOPILOT not in m,
             timeout=5, interval=0.05)
    start = len(trace.samples)
    motors = []
    wing_fly_to_contact(trace, timeout=200, each=lambda s: motors.append(fdm.motors.motors[0]))
    return trace.samples[start:], motors


def wing_contact_ground_m(fdm):
    c = fdm.model.contact
    return fdm.model.ground_at(c["e"], c["n"])


def wing_cut_failures(sitl, fdm):
    """After the contact the motor stops within WING_MOTOR_CUT_S and does not start again before the landing
    disarms."""
    m = fdm.model
    t0 = time.monotonic()
    cut_t = restart_t = disarm_t = None
    while time.monotonic() - t0 < 30.0 and disarm_t is None:
        now = time.monotonic()
        assert not m.crashed, f"crashed on the ground: {m.crash_reason}"
        if BOX_ARM not in sitl.modes():
            disarm_t = now
        elif fdm.motors.motors[0] <= 0.0:
            cut_t = cut_t or now
        elif cut_t is not None and restart_t is None:
            restart_t = now
        time.sleep(0.05)
    slid = math.hypot(m.pos[0] - m.contact["e"], m.pos[1] - m.contact["n"])
    log(f"motor stopped {(cut_t or now) - t0:.2f} s after the contact; slid {slid:.0f} m to "
        f"{m.ground_at() - m.ground_at(m.contact['e'], m.contact['n']):+.1f} m from the contact's height; disarmed "
        f"{'%.1f s' % (disarm_t - t0) if disarm_t else 'never'} after it")
    failures = []
    if cut_t is None or cut_t - t0 > WING_MOTOR_CUT_S:
        failures.append("the motor did not stop on the contact")
    if restart_t is not None:
        failures.append(f"the motor started again {restart_t - t0:.1f} s after the contact")
    if disarm_t is None:
        failures.append("still armed 30 s after the contact")
    return failures


def scenario_wing_descent_low_ground(sitl, rc, fdm):
    """The RC link goes with the AUTO-LAND procedure over a plain 30 m below the field it took off from, out of
    sight of home: not knowing the ground there, the wing flares over the field's height under power and circles on
    down until it meets the plain, the motor running, and cuts it on the contact, which is slow and nose up; it
    disarms once still."""
    samples, motors = wing_descent_fly(sitl, rc, fdm, 1300.0, WING_APPROACH_ALT_M)
    ground_m = wing_contact_ground_m(fdm)
    below = [m for s, m in zip(samples, motors) if s.u < -5.0]
    last = [m for s, m in zip(samples, motors) if s.t > samples[-1].t - 2.0]
    log(f"met the ground {ground_m:+.1f} m from the field's height; below the field the motor ran at "
        f"{min(below, default=0.0):.3f}..{max(below, default=0.0):.3f}, in the last 2 s before the contact up to "
        f"{max(last, default=0.0):.3f}")
    failures = []
    if ground_m > WING_LOW_GROUND_M + 1.0:
        failures.append(f"came down {ground_m:+.1f} m from the field's height, not on the plain")
    if not below or max(last, default=0.0) <= 0.0:
        failures.append("the motor stopped before the contact")
    assert_no_failures(failures + wing_landing_failures(sitl, fdm, motor_stop=False,
                                                        roll_deg=WING_UNKNOWN_GROUND_CONTACT_ROLL_DEG))


def scenario_wing_descent_high_ground(sitl, rc, fdm, sloped=False):
    """The RC link goes with the AUTO-LAND procedure over ground 15 m above the field it took off from, out of
    sight of home: not knowing the ground there, the wing circles down onto it at its descent's sink, banked gently
    enough not to strike a wing, cuts the motor on the contact, and keeps it stopped through the ground run until
    it disarms."""
    samples, motors = wing_descent_fly(sitl, rc, fdm, 450.0, 50.0 if sloped else WING_APPROACH_ALT_M)
    c = fdm.model.contact
    log(f"met the ground {wing_contact_ground_m(fdm):+.1f} m from the field's height, sinking {-c['vz']:.2f} m/s, "
        f"bank {c['roll']:+.1f} deg, {c['gs']:.1f} m/s over the ground")
    assert not fdm.model.crashed, f"crashed: {fdm.model.crash_reason}"
    assert_no_failures(wing_cut_failures(sitl, fdm))


def scenario_wing_descent_high_slope(sitl, rc, fdm):
    """As wing_descent_high_ground, onto ground rising 4 deg to the north."""
    scenario_wing_descent_high_ground(sitl, rc, fdm, sloped=True)


WING_LOW_GROUND_M = -30.0
WING_HIGH_GROUND_M = 15.0
WING_DESCENT_LOW_CFG = wing_descent_cfg(1700.0, WING_APPROACH_ALT_M)
WING_DESCENT_HIGH_CFG = wing_descent_cfg(700.0, WING_APPROACH_ALT_M)
# the default 2 m/s descent meeting a 4 deg rise at 15 m/s sinks onto it faster than the plant's 2.5 m/s crash
WING_DESCENT_SLOPE_CFG = wing_descent_cfg(700.0, 50.0, "set alt_hold_climb_rate = 10")

WING_WIND = {"model": "wing", "wind": (5.0, 0.0, 0.0)}   # 5 m/s from the west

SCENARIOS = {
    "baseline": (lambda s, r, f: boot_and_engage(s, r, f), []),
    # FIXED yaw: this scenario validates pure translation control; yaw-coupled
    # flight is mission_yaw's job (SITL's 15-20 Hz position loop wanders under
    # the default VELOCITY yaw during braking)
    "mission_flight": (scenario_mission_flight, ["set ap_yaw_mode = FIXED"]),
    "mission_yaw": (
        scenario_mission_yaw,
        # redirect the default waypoint east so the course demands a 90 deg swing
        [f"waypoint update 0 {HOME_LAT:.7f} {WP_EAST_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 none"],
    ),
    "mission_engage_backwards": (
        scenario_mission_engage_backwards,
        [
            # two north legs so the first is a pass-through carrot leg; the nose
            # starts backwards (initial_yaw_deg) in the default VELOCITY yaw mode
            f"waypoint update 0 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyby 0 none",
            f"waypoint insert 1 {WP_NORTH90_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyby 0 none",
        ],
        {"initial_yaw_deg": 180.0},
    ),
    "mission_corner": (
        scenario_mission_corner,
        [
            # 60 m north into the corner, then out on a ~130 deg turn
            f"waypoint update 0 {WP_NORTH60_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyby 0 none",
            f"waypoint insert 1 {WP_CORNER_LAT:.7f} {WP_CORNER_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyby 0 none",
        ],
    ),
    "mission_land": (
        scenario_mission_land,
        [
            "set ap_yaw_mode = FIXED",
            # short north leg, then LAND at an offset so the executor must fly
            # to the landing point before descending
            f"waypoint update 0 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 none",
            # 5 s pre-descent loiter at the LAND waypoint
            f"waypoint insert 1 {WP_NORTH40_LAT:.7f} {WP_EAST25_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 land 50 none",
            # SITL's 15-20 Hz position loop wanders during braking; a fatter
            # 3D gate keeps arrival deterministic
            "set ap_waypoint_hold_radius = 400",
            "set ap_landing_descent_rate = 200",  # keep the descent short
            "set landing_disarm_threshold = 10",   # jerk-based touchdown disarm
        ],
    ),
    "mission_takeoff": (
        scenario_mission_takeoff,
        [
            "set ap_yaw_mode = FIXED",
            # TAKEOFF's lat/lon are advisory; the climb happens in place
            f"waypoint update 0 {HOME_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 15) * 100)} 500 takeoff 0 none",
            f"waypoint insert 1 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 15) * 100)} 500 flyover 0 none",
        ],
    ),
    "mission_orbit": (
        scenario_mission_orbit,
        [
            "set ap_yaw_mode = FIXED",
            # 8 m pattern radius: caps the carrot at 2 m/s (0.25 rad/s), big
            # enough to be unambiguous against SITL's coarse position loop
            "set ap_waypoint_hold_radius = 800",
            f"waypoint update 0 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 none",
            f"waypoint insert 1 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 hold 600 orbit",
        ],
    ),
    "mission_figure8": (
        scenario_mission_figure8,
        [
            "set ap_yaw_mode = FIXED",
            "set ap_waypoint_hold_radius = 800",
            f"waypoint update 0 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 none",
            f"waypoint insert 1 {WP_NORTH40_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 hold 600 figure8",
        ],
    ),
    "mission_vert_rate": (
        scenario_mission_vert_rate,
        [
            "set ap_yaw_mode = FIXED",
            # 30 m climb at a leg-stated 1 m/s, over a 300 m leg so the climb finishes en route
            f"waypoint update 0 {WP_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 30) * 100)} 500 flyover 0 none 100 default",
        ],
    ),
    "mission_face_target": (
        scenario_mission_face_target,
        [
            f"waypoint update 0 {WP_NORTH90_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 none 0 face_target",
        ],
        {"initial_yaw_deg": 180.0},
    ),
    "mission_face_next": (
        scenario_mission_face_next,
        [
            f"waypoint update 0 {WP_NORTH90_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyby 0 none 0 face_next",
            f"waypoint insert 1 {WP_NORTH90_LAT:.7f} {WP_EAST90_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 500 flyover 0 none",
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
        [
            "set ap_max_distance_from_home = 50",
            "set ap_geofence_action = RTH",
            "set ap_yaw_mode = FIXED",
            "set gps_rescue_return_alt = 15",     # short climb keeps the return quick
            "set ap_waypoint_hold_radius = 400",
            "set ap_landing_descent_rate = 200",
            "set landing_disarm_threshold = 10",
        ],
    ),
    "geofence_rth_rxloss": (
        scenario_geofence_rth_rxloss,
        [
            "set ap_max_distance_from_home = 50",
            "set ap_geofence_action = RTH",
            "set ap_rx_loss_policy = CONTINUE",
            "rxfail 5 s 1000",  # stage 2 forces the AUTOPILOT switch low
            "set ap_yaw_mode = FIXED",
            "set gps_rescue_return_alt = 15",
            "set ap_waypoint_hold_radius = 400",
            "set ap_landing_descent_rate = 200",
            "set landing_disarm_threshold = 10",
        ],
    ),
    "rescue": (
        scenario_rescue_ab,
        RESCUE_CFG,
    ),
    "rescue_heading_recovery": (
        scenario_rescue_heading_recovery,
        [
            *RESCUE_CFG,
            "set mag_hardware = NONE",
            "set debug_mode = ATTITUDE",   # [4]: 1 while the IMU has no usable heading
            "set gps_rescue_return_alt = 15",
            "set gps_rescue_ascend_rate = 250",
        ],
    ),
    "rescue_heading_recovery_drift": (
        lambda sitl, rc, fdm: scenario_rescue_heading_recovery(sitl, rc, fdm, drift=True),
        [
            *RESCUE_CFG,
            "set mag_hardware = NONE",
            "set debug_mode = ATTITUDE",
            "set gps_rescue_return_alt = 15",
            "set gps_rescue_ascend_rate = 250",
        ],
    ),
    "rescue_near_home_no_heading": (
        scenario_rescue_near_home_no_heading,
        [*RESCUE_CFG, "set mag_hardware = NONE"],
    ),
    "rescue_fast_entry": (
        scenario_rescue_fast_entry,
        [
            *RESCUE_CFG,
            # a dash, not a cruise: the craft is still running from home when the
            # rescue takes over
            f"waypoint update 0 {WP_LAT:.7f} {HOME_LON:.7f} {int((HOME_ALT_M + 10) * 100)} 1500 flyover 0 none",
            "set ap_max_velocity = 1500",
            "set gps_rescue_return_alt = 15",
            "set gps_rescue_ascend_rate = 200",
        ],
    ),
    "rescue_near_home": (
        scenario_rescue_near_home,
        [*RESCUE_CFG, "set gps_rescue_min_start_dist = 10"],
    ),
    "rescue_gps_loss": (
        scenario_rescue_gps_loss,
        RESCUE_CFG,
    ),
    "rescue_switch_descent": (
        scenario_rescue_switch_descent,
        [*RESCUE_CFG, "aux 5 46 4 1700 2100 0 0"],   # BOXGPSRESCUE on AUX5
    ),
    "wing_turn": (scenario_wing_turn, WING_CFG, {"model": "wing"}),
    "wing_turn_wind": (scenario_wing_turn, WING_CFG, WING_WIND),
    "wing_althold": (scenario_wing_althold, WING_ALTHOLD_CFG, {"model": "wing"}),
    "wing_launch_into_althold": (scenario_wing_launch_into_althold, WING_ALTHOLD_CFG, {"model": "wing"}),
    "wing_launch_into_poshold": (scenario_wing_launch_into_poshold, WING_POSHOLD_CFG, {"model": "wing"}),
    "wing_loiter": (scenario_wing_loiter, WING_POSHOLD_CFG, {"model": "wing"}),
    "wing_loiter_no_mag": (scenario_wing_loiter, [*WING_POSHOLD_CFG, *WING_NO_MAG_CFG], {"model": "wing"}),
    "wing_loiter_wind": (scenario_wing_loiter_wind, WING_POSHOLD_CFG, WING_WIND),
    "wing_loiter_gps_loss": (scenario_wing_loiter_gps_loss, WING_POSHOLD_CFG, WING_WIND),
    "wing_mission": (scenario_wing_mission, WING_MISSION_CFG, {"model": "wing"}),
    "wing_mission_wind": (scenario_wing_mission, WING_MISSION_CFG, WING_WIND),
    "wing_rescue": (scenario_wing_rescue, WING_RESCUE_CFG, {"model": "wing"}),
    "wing_rescue_wind": (scenario_wing_rescue_wind, WING_RESCUE_CFG, WING_WIND),
    "wing_rescue_gps_loss": (scenario_wing_rescue_gps_loss, WING_RESCUE_CFG, {"model": "wing"}),
    "wing_rescue_armed_in_hand": (
        scenario_wing_rescue_armed_in_hand,
        [*WING_RESCUE_CFG, "set ap_wing_land_launch_height = 150"],
        {"model": "wing"},
    ),
    "wing_rx_loss_rescue": (
        scenario_wing_rx_loss_rescue,
        [*WING_RESCUE_CFG, "set failsafe_procedure = GPS-RESCUE"],
        {"model": "wing"},
    ),
    "wing_rx_loss_continue": (scenario_wing_rx_loss_continue, WING_RX_CONTINUE_CFG, {"model": "wing"}),
    "wing_rx_loss_land": (scenario_wing_rx_loss_land, WING_RX_LAND_CFG, {"model": "wing"}),
    "wing_descent_low_ground": (
        scenario_wing_descent_low_ground, WING_DESCENT_LOW_CFG,
        {"model": "wing", "ground": ground_north(WING_LOW_GROUND_M, 150.0, 300.0)},
    ),
    "wing_descent_high_ground": (
        scenario_wing_descent_high_ground, WING_DESCENT_HIGH_CFG,
        {"model": "wing", "ground": ground_north(WING_HIGH_GROUND_M, 150.0, 250.0)},
    ),
    "wing_descent_high_slope": (
        scenario_wing_descent_high_slope, WING_DESCENT_SLOPE_CFG,
        {"model": "wing", "ground": ground_north(WING_HIGH_GROUND_M, 150.0, 250.0, math.tan(math.radians(4.0)))},
    ),
    "wing_autoland": (scenario_wing_autoland, WING_AUTOLAND_CFG, {"model": "wing"}),
    "wing_autoland_no_mag": (scenario_wing_autoland, [*WING_AUTOLAND_CFG, *WING_NO_MAG_CFG], {"model": "wing"}),
    "wing_autoland_crosswind": (scenario_wing_autoland_crosswind, WING_AUTOLAND_CFG, WING_WIND),
    "wing_autoland_headwind": (
        scenario_wing_autoland_headwind, WING_AUTOLAND_NORTH_CFG, {"model": "wing", "wind": (0.0, -8.0, 0.0)},
    ),
    "wing_autoland_tailwind_flip": (
        scenario_wing_autoland_tailwind_flip, WING_AUTOLAND_CFG, {"model": "wing", "wind": (0.0, -5.0, 0.0)},
    ),
    "wing_go_around_stick": (scenario_wing_go_around_stick, WING_AUTOLAND_CFG, {"model": "wing"}),
    "wing_go_around_baro": (scenario_wing_go_around_baro, WING_AUTOLAND_CFG, {"model": "wing"}),
    "wing_go_around_gps": (scenario_wing_go_around_gps, WING_AUTOLAND_CFG, {"model": "wing"}),
    "wing_mission_land": (scenario_wing_mission_land, WING_MISSION_LAND_CFG, {"model": "wing"}),
}


def decode_blackbox_logs(scenario_dir):
    """Best-effort: decode .BFL artifacts when blackbox_decode is available.
    Never gates pass/fail — the trajectory recorder is the authority."""
    if not shutil.which("blackbox_decode"):
        return
    for entry in sorted(os.listdir(scenario_dir)):
        if entry.upper().endswith(".BFL"):
            subprocess.run(["blackbox_decode", os.path.join(scenario_dir, entry)],
                           capture_output=True, check=False)


def run_leg(name, variant, body, extra_cfg, opts, binary, leg_dir):
    os.makedirs(leg_dir)
    sitl = Sitl(binary, leg_dir)
    rc = motors = fdm = poller = pwm_raw = None
    try:
        # feed construction can fail (port 9002 bind); it must fail the
        # scenario, not abort the suite
        rc = RcFeed()
        motors = MotorFeed()
        poller = StatusPoller(sitl)
        is_wing = opts.get("model") == "wing"
        if is_wing:
            pwm_raw = PwmRawFeed()
        fdm = FdmFeed(motors, initial_yaw_deg=opts.get("initial_yaw_deg", 0.0), status=poller,
                      model=WingMotionModel(opts.get("wind", (0.0, 0.0, 0.0)), opts.get("ground")) if is_wing else None,
                      pwm_raw=pwm_raw, sensors=SensorErrors() if is_wing else None)
        sitl.provision(base_config(extra_cfg))
        sitl.start()
        motors.start()
        if pwm_raw:
            pwm_raw.start()
        poller.start()
        if variant is None:
            return body(sitl, rc, fdm)
        return body(sitl, rc, fdm, variant)
    finally:
        for feed in (rc, fdm, motors, pwm_raw, poller):
            if feed is not None:
                feed.shutdown()
        sitl.stop()
        decode_blackbox_logs(leg_dir)


def run_scenario(name, binary, workdir, binary_b=None, binary_wing=None):
    spec = SCENARIOS[name]
    body, extra_cfg = spec[0], spec[1]
    opts = spec[2] if len(spec) > 2 else {}
    scenario_dir = os.path.join(workdir, name)
    shutil.rmtree(scenario_dir, ignore_errors=True)
    os.makedirs(scenario_dir)

    log(f"=== scenario: {name}")
    if opts.get("model") == "wing":
        if binary_wing is None:
            log(f"=== SKIP: {name} (wing scenario, no --binary-wing)")
            return None
        binary = binary_wing
    try:
        if opts.get("ab"):
            if binary_b is None:
                log(f"=== SKIP: {name} (A/B scenario, no --binary-b)")
                return None
            metrics_a = run_leg(name, "A", body, extra_cfg, opts, binary, os.path.join(scenario_dir, "A"))
            metrics_b = run_leg(name, "B", body, extra_cfg, opts, binary_b, os.path.join(scenario_dir, "B"))
            opts["compare"](metrics_a, metrics_b)
        else:
            run_leg(name, None, body, extra_cfg, opts, binary, os.path.join(scenario_dir, "run"))
        log(f"=== PASS: {name}")
        return True
    except (AssertionError, RuntimeError, TimeoutError, OSError) as e:
        log(f"=== FAIL: {name}: {e}")
        return False


def main():
    global VERBOSE, TELEMETRY_PORT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--binary", required=True, help="path to betaflight_SITL.elf (built with USE_FLIGHT_PLAN)")
    ap.add_argument("--binary-b", help="rescue-plan binary (-DENABLE_RESCUE_PLAN=1) for A/B scenarios")
    ap.add_argument("--binary-wing", help="wing binary (-DUSE_WING) for fixed-wing scenarios")
    ap.add_argument("--scenario", default="all", choices=["all"] + list(SCENARIOS))
    ap.add_argument("--workdir", default="/tmp/sitl_harness")
    ap.add_argument("--telemetry-port", type=int, default=TELEMETRY_PORT,
                    help="UDP port for ground-truth JSON telemetry (0 disables)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    VERBOSE = args.verbose
    TELEMETRY_PORT = args.telemetry_port

    os.makedirs(args.workdir, exist_ok=True)
    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    results = {name: run_scenario(name, args.binary, args.workdir, args.binary_b, args.binary_wing)
               for name in names}

    log("--- summary")
    for name, ok in results.items():
        log(f"{'PASS' if ok else 'SKIP' if ok is None else 'FAIL'}  {name}")
    sys.exit(0 if all(ok is not False for ok in results.values()) else 1)


if __name__ == "__main__":
    main()
