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
MSP_BOXIDS = 119
MSP_ACC_CALIBRATION = 205

TCP_PORT = 5761
RC_PORT = 9004
FDM_PORT = 9003

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


class FdmFeed(threading.Thread):
    """50 Hz fdm_packet stream: level attitude, still air, virtual GPS position.

    The default SITL build is the Gazebo bridge, whose quaternion path is
    q' = Rz(pi/2) * q_plugin; send q_plugin = (k, 0, 0, -k) so the FC sees
    identity attitude. Position moves are teleports — good enough for the
    state-machine scenarios exercised here.
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lat = HOME_LAT
        self.lon = HOME_LON
        self.alt = HOME_ALT_M
        self.running = True
        self.t0 = time.monotonic()

    def move_east(self, metres):
        self.lon += metres / (M_PER_DEG * math.cos(math.radians(self.lat)))

    def run(self):
        k = math.sqrt(0.5)
        while self.running:
            pkt = struct.pack(
                "<18d",
                time.monotonic() - self.t0,
                0.0, 0.0, 0.0,              # gyro rad/s
                0.0, 0.0, -9.80665,         # accel m/s^2 (level, 1G)
                k, 0.0, 0.0, -k,            # quat w,x,y,z (identity after bridge transform)
                0.0, 0.0, 0.0,              # velocity ENU m/s
                self.lon, self.lat, self.alt,  # virtual GPS position
                101325.0,                   # pressure (unused by gazebo bridge)
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
            self.proc = subprocess.Popen([self.binary], cwd=self.workdir, stdout=logf, stderr=logf)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if self.proc.poll() is not None:
                    raise RuntimeError(f"SITL exited early (rc={self.proc.returncode}); see sitl.log")
                try:
                    self.sock = socket.create_connection(("127.0.0.1", TCP_PORT), timeout=1)
                    self.msp = Msp(self.sock)
                    self.boxids = list(self.msp.request(MSP_BOXIDS))
                    debug(f"boxids: {self.boxids}")
                    return
                except OSError:
                    time.sleep(0.3)
            debug(f"MSP port never appeared (attempt {attempt + 1}); relaunching")
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


def base_config(extra):
    dlat = 300.0 / M_PER_DEG  # waypoint 300 m north of home
    wp_lat = HOME_LAT + dlat
    return [
        "feature GPS",
        "set gps_provider = VIRTUAL",
        "set failsafe_procedure = AUTO-LAND",
        "set failsafe_delay = 10",
        "set small_angle = 180",
        "aux 0 0 0 1700 2100 0 0",   # ARM on AUX1
        "aux 1 56 1 1700 2100 0 0",  # AUTOPILOT on AUX2
        f"waypoint insert 0 {wp_lat:.7f} {HOME_LON:.7f} 6000 300 flyover 0 orbit",
    ] + extra


def boot_and_engage(sitl, rc, fdm):
    """Common preamble: boot, GPS fix, arm, raise throttle, engage AUTOPILOT."""
    rc.start()
    fdm.start()

    wait_for("GPS fix + RX recovery (arming flags clear)", lambda: sitl.status()["arming_flags"] == 0, timeout=40)

    rc.set(4, RC_HIGH)  # AUX1: arm (throttle is low)
    wait_for("armed", lambda: BOX_ARM in sitl.modes())

    rc.set(2, 1600)     # raise throttle (wasThrottleRaised)
    time.sleep(1.0)

    rc.set(5, RC_HIGH)  # AUX2: AUTOPILOT
    modes = wait_for(
        "AUTOPILOT + ALTHOLD + POSHOLD active (mode wiring)",
        lambda: sitl.modes() >= {BOX_AUTOPILOT, BOX_ALTHOLD, BOX_POSHOLD} and sitl.modes() or None,
    )
    return modes


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
        assert BOX_AUTOPILOT in sitl.modes(), "mission dropped during CONTINUE"
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
    log(f"teleporting 150 m east to breach 50 m geofence (action={action})")
    fdm.move_east(150.0)

    if action == "RTH":
        wait_for(
            "GPS rescue engaged, mission disengaged",
            lambda: (lambda m: BOX_GPSRESCUE in m and BOX_AUTOPILOT not in m)(sitl.modes()),
        )
    else:  # LAND
        time.sleep(3)
        modes = sitl.modes()
        assert BOX_AUTOPILOT in modes, f"mission dropped instead of landing: {modes}"
        assert BOX_GPSRESCUE not in modes, f"unexpected rescue: {modes}"
        log("mission holds LANDING state at current position")


SCENARIOS = {
    "baseline": (lambda s, r, f: boot_and_engage(s, r, f), []),
    "rx_disable": (lambda s, r, f: scenario_rx_loss(s, r, f, "DISABLE"), ["set ap_rx_loss_policy = DISABLE"]),
    "rx_continue": (lambda s, r, f: scenario_rx_loss(s, r, f, "CONTINUE"), ["set ap_rx_loss_policy = CONTINUE"]),
    "rx_land": (lambda s, r, f: scenario_rx_loss(s, r, f, "LAND"), ["set ap_rx_loss_policy = LAND"]),
    "geofence_land": (
        lambda s, r, f: scenario_geofence(s, r, f, "LAND"),
        ["set ap_max_distance_from_home = 50", "set ap_geofence_action = LAND"],
    ),
    "geofence_rth": (
        lambda s, r, f: scenario_geofence(s, r, f, "RTH"),
        ["set ap_max_distance_from_home = 50", "set ap_geofence_action = RTH"],
    ),
}


def run_scenario(name, binary, workdir):
    body, extra_cfg = SCENARIOS[name]
    scenario_dir = os.path.join(workdir, name)
    shutil.rmtree(scenario_dir, ignore_errors=True)
    os.makedirs(scenario_dir)

    log(f"=== scenario: {name}")
    sitl = Sitl(binary, scenario_dir)
    rc = RcFeed()
    fdm = FdmFeed()
    try:
        sitl.provision(base_config(extra_cfg))
        sitl.start()
        body(sitl, rc, fdm)
        log(f"=== PASS: {name}")
        return True
    except (AssertionError, RuntimeError, TimeoutError) as e:
        log(f"=== FAIL: {name}: {e}")
        return False
    finally:
        rc.shutdown()
        fdm.shutdown()
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
