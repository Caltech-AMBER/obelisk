#!/usr/bin/env python3
"""
THEMIS Obelisk UDP Server — runs on the robot's onboard PC.

Bridges the C++ ThemisInterface (Obelisk-side, on a dev machine) to AOS's
Python control APIs (wbc_api, lm_api, memory_manager) over UDP.

Two operating modes selected at startup:

  --mode user-ctrl  (default)
      Joint-level control. Kills onboard locomotion/manipulation/command
      threads; the RL policy or external controller has direct authority
      over all 28 joints via wbc_api.set_joint_states(). Mirrors the
      original sim2real_server.py from themis_rl.

  --mode high-level
      Velocity control. Kills only the command thread; keeps locomotion
      and manipulation alive. The desktop sends base velocity / orientation
      / CoM commands, which the server forwards to AOS via lm_api.
      walking_mode is toggled via lm_api.set_locomotion_mode.

Wire protocol (byte 0 = message type; remainder = payload):

  desktop → robot:
    0x05 FULL_STATE_REQUEST   empty
    0x07 FSM_TRANSITION       1 byte: ThemisFSMState enum value
    0x14 BASE_VEL_CMD         3 doubles (vx, vy, wz)        [high-level mode only]
    0x15 BASE_ORIENT_CMD      3 doubles (roll, pitch, yaw)  [high-level mode only]
    0x16 FULL_BODY_CMD        140 doubles                   [user-ctrl mode only]
                                q[28] dq[28] u[28] kp[28] kd[28]
    0x18 COM_POS_CMD          3 doubles (x, y, z)           [high-level mode only]
    0x20 HEARTBEAT            empty

  robot → desktop:
    0x06 FULL_STATE_RESPONSE  100 doubles
                                q[28] dq[28] u[28]
                                imu_accel[3] imu_gyro[3] imu_rot[9] timestamp[1]
    0xFE ACK                  1 byte status (0x00 ok, 0x01 rejected)

ThemisFSMState (matches themis_fsm.h):
    0 INIT             motors disabled
    1 USER_POSE        client-side ramp; server holds previous state
    2 USER_CTRL        client sends FULL_BODY_CMD                 [user-ctrl mode]
    3 HIGH_LEVEL_VEL   client sends BASE_VEL_CMD via lm_api       [high-level mode]
    4 DAMPING          wbc_api.damping_motors per chain
    5 ESTOP            sticky DAMPING

Joint ordering (28 joints):
    [right_leg(6), left_leg(6), right_arm(7), left_arm(7), head(2)]
"""

# Copyright (c) 2026 California Institute of Technology
#
# This file imports AOS modules (Play.Others.wbc, Play.Others.locomotion,
# Startup.memory_manager) which are licensed under GPL-3.0. As a derivative
# work of those modules, this file is distributed under GPL-3.0. See
# https://www.gnu.org/licenses/gpl-3.0.html for the full license text.

import argparse
import os
import socket
import struct
import subprocess
import sys
import time
import traceback
from enum import IntEnum

import numpy as np

# ── AOS imports ──────────────────────────────────────────────────────────────
AOS_PATH = os.environ.get("AOS_PATH", "/home/themis/THEMIS/THEMIS")
if AOS_PATH not in sys.path:
    sys.path.insert(0, os.path.dirname(AOS_PATH))
    sys.path.insert(0, AOS_PATH)

try:
    from Play.Others import wbc as wbc_api
    print("[server] OK   wbc_api imported")
except ImportError:
    print(f"[server] FAIL wbc_api not available (AOS_PATH={AOS_PATH}). Cannot run.")
    sys.exit(1)

try:
    from Play.Others import locomotion as lm_api
    print("[server] OK   lm_api imported")
except ImportError:
    print(f"[server] FAIL lm_api not available. Cannot run high-level mode.")
    lm_api = None

try:
    import Startup.memory_manager as MM
    print("[server] OK   memory_manager imported")
except ImportError:
    print("[server] WARN memory_manager not available")
    MM = None


# ── Wire protocol ────────────────────────────────────────────────────────────
MSG_FULL_STATE_REQUEST  = 0x05
MSG_FULL_STATE_RESPONSE = 0x06
MSG_FSM_TRANSITION      = 0x07
MSG_BASE_VEL_CMD        = 0x14
MSG_BASE_ORIENT_CMD     = 0x15
MSG_FULL_BODY_CMD       = 0x16
MSG_COM_POS_CMD         = 0x18
MSG_HEARTBEAT           = 0x20
MSG_ACK                 = 0xFE

ACK_OK       = 0x00
ACK_REJECTED = 0x01

NUM_JOINTS = 28


class FSMState(IntEnum):
    INIT           = 0
    USER_POSE      = 1
    USER_CTRL      = 2
    HIGH_LEVEL_VEL = 3
    DAMPING        = 4
    ESTOP          = 5


# ── AOS chain mapping ────────────────────────────────────────────────────────
# 28-joint slot layout → AOS chain id + slice into the 28-joint array.
# Order: [right_leg(6), left_leg(6), right_arm(7), left_arm(7), head(2)]
CHAIN_HEAD       =  0
CHAIN_RIGHT_LEG  = +1
CHAIN_LEFT_LEG   = -1
CHAIN_RIGHT_ARM  = +2
CHAIN_LEFT_ARM   = -2

CHAINS = [
    (CHAIN_RIGHT_LEG, 0,  6),
    (CHAIN_LEFT_LEG,  6,  12),
    (CHAIN_RIGHT_ARM, 12, 19),
    (CHAIN_LEFT_ARM,  19, 26),
    (CHAIN_HEAD,      26, 28),
]


# ── Screen-process management ────────────────────────────────────────────────
def _kill_screen_window(session: str, window: str) -> None:
    """Send Ctrl-C to a `screen` window. Retries 5× because AOS threads
    sometimes ignore the first signal during their startup sequence."""
    for _ in range(5):
        try:
            subprocess.run(
                ["screen", "-S", session, "-p", window, "-X", "stuff", "\x03"],
                timeout=2,
                capture_output=True,
            )
        except Exception:
            pass
        time.sleep(0.1)


def kill_threads_for_user_ctrl() -> None:
    """user-ctrl mode: take exclusive joint-level authority over all 28 joints.

    Kills:
      - locomotion   (would otherwise overwrite our joint setpoints from MM.USER_COMMAND)
      - manipulation (same reason, for arms)
      - command      (was running command_manager.py, which writes to MM.USER_COMMAND)
    Keeps:
      - control      (the WBC binary that consumes MM.*_JOINT_COMMAND and writes to BEAR actuators)
    """
    for name in ("locomotion", "manipulation", "command"):
        print(f"[server] killing screen window '{name}' ...")
        _kill_screen_window("themis", name)
        print(f"[server] OK   '{name}' killed")

    # Belt-and-suspenders: clear thread-command SHM flags as well.
    if MM is not None:
        for key in ("locomotion", "manipulation", "command"):
            try:
                MM.THREAD_COMMAND.set({key: np.array([0.0])}, opt="update")
            except Exception:
                pass

    time.sleep(1.0)
    print("[server] all conflicting onboard threads killed (user-ctrl mode)")


def kill_threads_for_high_level() -> None:
    """high-level mode: keep locomotion alive, only kill the user-command thread.

    The original command_manager.py reads from gamepad/keyboard and writes to
    MM.USER_COMMAND. We need to be the only writer, so we kill it.
    """
    print("[server] killing screen window 'command' ...")
    _kill_screen_window("themis", "command")
    print("[server] OK   'command' killed")

    if MM is not None:
        try:
            MM.THREAD_COMMAND.set({"command": np.array([0.0])}, opt="update")
        except Exception:
            pass

    time.sleep(1.0)
    print("[server] command thread killed (high-level mode); locomotion + manipulation still running")


# ── State packet construction ────────────────────────────────────────────────
def read_full_state() -> bytes:
    """Build a FULL_STATE_RESPONSE packet.

    Layout (1 byte type + 100 doubles = 801 bytes):
      msg_type(1) + q[28] + dq[28] + u[28] + accel[3] + gyro[3] + R[9] + ts[1]

    Note: wbc_api.get_joint_states() returns (q, dq, u) — a 3-tuple in AOS v0.2.4
    despite the dev manual claiming (q, dq, u, t, v). Temps + voltages are not
    exposed; the wire format leaves no slot for them.
    """
    joint_q  = np.zeros(NUM_JOINTS, dtype=np.float64)
    joint_dq = np.zeros(NUM_JOINTS, dtype=np.float64)
    joint_u  = np.zeros(NUM_JOINTS, dtype=np.float64)

    for chain_id, start, end in CHAINS:
        n = end - start
        try:
            q, dq, u = wbc_api.get_joint_states(chain_id)
        except Exception:
            continue
        joint_q[start:end]  = np.asarray(q,  dtype=np.float64).ravel()[:n]
        joint_dq[start:end] = np.asarray(dq, dtype=np.float64).ravel()[:n]
        joint_u[start:end]  = np.asarray(u,  dtype=np.float64).ravel()[:n]

    try:
        imu_accel, imu_gyro, imu_R = wbc_api.get_imu_states()
        imu_accel = np.asarray(imu_accel, dtype=np.float64).ravel()[:3]
        imu_gyro  = np.asarray(imu_gyro,  dtype=np.float64).ravel()[:3]
        imu_R     = np.asarray(imu_R,     dtype=np.float64).ravel()[:9]
    except Exception:
        imu_accel = np.zeros(3, dtype=np.float64)
        imu_gyro  = np.zeros(3, dtype=np.float64)
        imu_R     = np.eye(3, dtype=np.float64).ravel()

    buf = struct.pack("B", MSG_FULL_STATE_RESPONSE)
    buf += joint_q.tobytes()
    buf += joint_dq.tobytes()
    buf += joint_u.tobytes()
    buf += imu_accel.tobytes()
    buf += imu_gyro.tobytes()
    buf += imu_R.tobytes()
    buf += np.array([time.time()], dtype=np.float64).tobytes()
    return buf


# ── Command handlers ─────────────────────────────────────────────────────────
def handle_full_body_cmd(payload: bytes) -> bool:
    """Apply a 140-double FULL_BODY_CMD via wbc_api.set_joint_states() per chain.

    Layout: q[28] + dq[28] + u[28] + kp[28] + kd[28].
    """
    data = np.frombuffer(payload, dtype=np.float64)
    if data.size != 5 * NUM_JOINTS:
        print(f"[server] FULL_BODY_CMD wrong size: {data.size} (expected {5 * NUM_JOINTS})")
        return False

    q  = data[0 * NUM_JOINTS : 1 * NUM_JOINTS].copy()
    dq = data[1 * NUM_JOINTS : 2 * NUM_JOINTS].copy()
    u  = data[2 * NUM_JOINTS : 3 * NUM_JOINTS].copy()
    kp = data[3 * NUM_JOINTS : 4 * NUM_JOINTS].copy()
    kd = data[4 * NUM_JOINTS : 5 * NUM_JOINTS].copy()

    for chain_id, start, end in CHAINS:
        try:
            wbc_api.set_joint_states(
                chain_id,
                u[start:end],
                q[start:end],
                dq[start:end],
                kp[start:end],
                kd[start:end],
            )
        except Exception as e:
            print(f"[server] wbc_api.set_joint_states(chain={chain_id}) failed: {e}")
            return False
    return True


def handle_base_vel_cmd(payload: bytes) -> bool:
    if len(payload) != 3 * 8:
        return False
    vx, vy, wz = struct.unpack("ddd", payload)
    try:
        lm_api.set_walking_velocity(np.array([vx, vy]), wz)
        return True
    except Exception as e:
        print(f"[server] lm_api.set_walking_velocity failed: {e}")
        return False


def handle_base_orient_cmd(payload: bytes) -> bool:
    if len(payload) != 3 * 8:
        return False
    roll, pitch, yaw = struct.unpack("ddd", payload)
    try:
        lm_api.set_base_orientation(roll, pitch, yaw)
        return True
    except Exception as e:
        print(f"[server] lm_api.set_base_orientation failed: {e}")
        return False


def handle_com_pos_cmd(payload: bytes) -> bool:
    if len(payload) != 3 * 8:
        return False
    x, y, z = struct.unpack("ddd", payload)
    try:
        lm_api.set_com_position(x, y, z)
        return True
    except Exception as e:
        print(f"[server] lm_api.set_com_position failed: {e}")
        return False


def handle_fsm_transition(payload: bytes, mode: str, current_state: FSMState) -> FSMState:
    """Apply an FSM transition. Returns the new state (may equal current_state if rejected).

    State-entry actions:
      INIT            disable all motor chains (safe state)
      USER_POSE       leaving HIGH_LEVEL: zero walking velocity, then set locomotion mode 0
                      (per §3.6.1 of the dev manual). Otherwise no-op — client drives the ramp.
      USER_CTRL       no-op; FULL_BODY_CMD packets now flow
      HIGH_LEVEL_VEL  set locomotion mode 1 (walking). Rejected if mode != high-level.
      DAMPING         wbc_api.damping_motors per chain
      ESTOP           wbc_api.damping_motors per chain; sticky
    """
    if len(payload) < 1:
        return current_state

    try:
        new_state = FSMState(payload[0])
    except ValueError:
        print(f"[server] unknown FSM state value: {payload[0]}")
        return current_state

    # ESTOP is sticky.
    if current_state == FSMState.ESTOP and new_state != FSMState.ESTOP:
        print("[server] rejecting transition out of ESTOP")
        return current_state

    if new_state == FSMState.INIT:
        for chain_id, _, _ in CHAINS:
            try:
                wbc_api.disable_motors(chain_id)
            except Exception as e:
                print(f"[server] disable_motors(chain={chain_id}) failed: {e}")

    elif new_state == FSMState.USER_POSE:
        if current_state == FSMState.HIGH_LEVEL_VEL and lm_api is not None:
            # Manual §3.6.1: zero velocity before switching to standing.
            try:
                lm_api.set_walking_velocity(np.array([0.0, 0.0]), 0.0)
                lm_api.set_locomotion_mode(0)
            except Exception as e:
                print(f"[server] failed to zero/stand on HIGH_LEVEL_VEL → USER_POSE: {e}")
                return current_state

    elif new_state == FSMState.USER_CTRL:
        if mode != "user-ctrl":
            print(f"[server] rejecting USER_CTRL transition: server started in --mode={mode}")
            return current_state

    elif new_state == FSMState.HIGH_LEVEL_VEL:
        if mode != "high-level":
            print(f"[server] rejecting HIGH_LEVEL_VEL transition: server started in --mode={mode}")
            return current_state
        if lm_api is None:
            print("[server] rejecting HIGH_LEVEL_VEL: lm_api unavailable")
            return current_state
        try:
            lm_api.set_locomotion_mode(1)
        except Exception as e:
            print(f"[server] set_locomotion_mode(1) failed: {e}")
            return current_state

    elif new_state in (FSMState.DAMPING, FSMState.ESTOP):
        # Damping is supported for head + legs + arms (chain ∈ {0, ±1, ±2}); hands not supported.
        for chain_id, _, _ in CHAINS:
            try:
                wbc_api.damping_motors(chain_id)
            except Exception as e:
                print(f"[server] damping_motors(chain={chain_id}) failed: {e}")

    print(f"[server] FSM {current_state.name} → {new_state.name}")
    return new_state


# ── Main loop ────────────────────────────────────────────────────────────────
def run_server(host: str, port: int, mode: str) -> None:
    print("=" * 64)
    print(f"  THEMIS Obelisk UDP Server  (mode={mode})")
    print("=" * 64)
    print(f"  AOS_PATH: {AOS_PATH}")
    print(f"  bind:     {host}:{port}")
    print("=" * 64)

    if MM is not None:
        try:
            MM.connect()
            print("[server] shared memory connected")
        except Exception as e:
            print(f"[server] WARN MM.connect() failed: {e}")

    # Verify wbc_api before killing anything.
    try:
        q, dq, u = wbc_api.get_joint_states(CHAIN_RIGHT_LEG)
        print(f"[server] wbc_api OK; right leg q = {np.degrees(np.asarray(q)).round(1)}")
    except Exception as e:
        print(f"[server] FAIL wbc_api.get_joint_states(): {e}")
        traceback.print_exc()
        sys.exit(1)

    if mode == "user-ctrl":
        kill_threads_for_user_ctrl()
    elif mode == "high-level":
        if lm_api is None:
            print("[server] FAIL high-level mode requires lm_api")
            sys.exit(1)
        kill_threads_for_high_level()
    else:
        print(f"[server] FAIL unknown mode: {mode}")
        sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.settimeout(0.5)
    print(f"\n[server] listening on {host}:{port}")
    print("[server] waiting for desktop connection ...\n")

    last_heartbeat = time.time()
    pkt_count = 0
    cmd_count = 0
    last_stats = time.time()
    state = FSMState.INIT

    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                if time.time() - last_heartbeat > 30.0:
                    print("[server] no heartbeat for 30s — still waiting ...")
                    last_heartbeat = time.time()
                continue

            if len(data) < 1:
                continue

            msg_type = data[0]
            payload = data[1:]
            ack_status = ACK_OK

            if msg_type == MSG_FULL_STATE_REQUEST:
                sock.sendto(read_full_state(), addr)

            elif msg_type == MSG_FULL_BODY_CMD:
                if mode != "user-ctrl" or state != FSMState.USER_CTRL:
                    ack_status = ACK_REJECTED
                else:
                    if handle_full_body_cmd(payload):
                        cmd_count += 1
                    else:
                        ack_status = ACK_REJECTED

            elif msg_type == MSG_BASE_VEL_CMD:
                if mode != "high-level" or state != FSMState.HIGH_LEVEL_VEL:
                    ack_status = ACK_REJECTED
                else:
                    if handle_base_vel_cmd(payload):
                        cmd_count += 1
                    else:
                        ack_status = ACK_REJECTED

            elif msg_type == MSG_BASE_ORIENT_CMD:
                if mode != "high-level" or state not in (FSMState.USER_POSE, FSMState.INIT):
                    # Manual §3.6.1: do not adjust base orientation while walking.
                    ack_status = ACK_REJECTED
                else:
                    if not handle_base_orient_cmd(payload):
                        ack_status = ACK_REJECTED

            elif msg_type == MSG_COM_POS_CMD:
                if mode != "high-level" or state not in (FSMState.USER_POSE, FSMState.INIT):
                    ack_status = ACK_REJECTED
                else:
                    if not handle_com_pos_cmd(payload):
                        ack_status = ACK_REJECTED

            elif msg_type == MSG_FSM_TRANSITION:
                state = handle_fsm_transition(payload, mode, state)

            elif msg_type == MSG_HEARTBEAT:
                last_heartbeat = time.time()
                sock.sendto(struct.pack("BB", MSG_ACK, ACK_OK), addr)

            else:
                # Unknown msg type — drop silently.
                pass

            pkt_count += 1

            if time.time() - last_stats >= 5.0:
                elapsed = time.time() - last_stats
                print(f"[server] {pkt_count/elapsed:.0f} pkt/s | {cmd_count/elapsed:.0f} cmd/s | state={state.name}")
                pkt_count = 0
                cmd_count = 0
                last_stats = time.time()

    except KeyboardInterrupt:
        print("\n[server] shutting down ...")
    finally:
        sock.close()
        # On clean shutdown, leave everything in damping. ESTOP is left to the user.
        for chain_id, _, _ in CHAINS:
            try:
                wbc_api.damping_motors(chain_id)
            except Exception:
                pass
        print("[server] motors set to damping. exit.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="0.0.0.0", help="UDP bind address (default 0.0.0.0)")
    parser.add_argument("--port", type=int, default=9870, help="UDP port (default 9870)")
    parser.add_argument(
        "--mode",
        choices=("user-ctrl", "high-level"),
        default="user-ctrl",
        help="Authority mode. user-ctrl = direct joint control via wbc_api (kills locomotion); "
             "high-level = velocity commands via lm_api (keeps locomotion).",
    )
    args = parser.parse_args()
    run_server(args.host, args.port, args.mode)


if __name__ == "__main__":
    main()
