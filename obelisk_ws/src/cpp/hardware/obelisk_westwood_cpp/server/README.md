# themis_obk_server — on-robot UDP bridge

This Python service runs on the **THEMIS main PC**, not on the Obelisk dev
machine. It translates UDP packets from the C++ `ThemisInterface` into AOS
API calls (`wbc_api`, `lm_api`, `memory_manager`).

The wire protocol is owned by [themis_interface.h](../../obelisk/cpp/hardware/robots/westwood/themis_interface.h) — the byte
layouts and message-type constants there are the contract; this file
implements the server end.

## Installation on the robot

A fresh-from-factory THEMIS already has AOS installed at
`/home/themis/THEMIS/THEMIS/`. On your dev machine:

```
scp themis_obk_server.py themis@<robot-ip>:/home/themis/THEMIS/THEMIS/Play/Others/
```

Then on the robot:

```
ssh themis@<robot-ip>
cd ~/THEMIS/THEMIS
screen -S themis -X screen -t obk_server   # add a new window to the AOS screen session
screen -S themis -p obk_server -X stuff 'python3 -m Play.Others.themis_obk_server --mode user-ctrl\n'
```

Or just run it directly in a new terminal once AOS is up:

```
python3 -m Play.Others.themis_obk_server --mode user-ctrl
```

## Modes

| Mode | Behavior |
|---|---|
| `--mode user-ctrl` (default) | Joint-level control. Kills `locomotion`, `manipulation`, `command` screen windows; keeps `control` running. Accepts `FULL_BODY_CMD` (28-joint q/dq/u/kp/kd) and routes to `wbc_api.set_joint_states` per chain. |
| `--mode high-level` | Velocity control. Kills only `command`; keeps `locomotion` + `manipulation` running. Accepts `BASE_VEL_CMD` (vx, vy, wz) and routes to `lm_api.set_walking_velocity`. Also accepts base orientation and CoM position commands (but only while in `USER_POSE` per the dev manual's caveat about not adjusting them while walking). |

## CLI

```
python3 -m Play.Others.themis_obk_server \
    [--host 0.0.0.0]      # bind address (default 0.0.0.0)
    [--port 9870]         # UDP port
    [--mode user-ctrl]    # or high-level
```

`AOS_PATH` env var overrides the AOS install location (default `/home/themis/THEMIS/THEMIS`).

## Wire protocol

| Byte | Direction | Payload | Notes |
|---|---|---|---|
| `0x05 FULL_STATE_REQUEST` | desktop → robot | empty | Server responds with `0x06` |
| `0x06 FULL_STATE_RESPONSE` | robot → desktop | 100 doubles | `q[28] dq[28] u[28] accel[3] gyro[3] R[9] ts[1]` |
| `0x07 FSM_TRANSITION` | desktop → robot | 1 byte | `ThemisFSMState` enum (0–5) |
| `0x14 BASE_VEL_CMD` | desktop → robot | 3 doubles | `vx, vy, wz` (high-level mode only) |
| `0x15 BASE_ORIENT_CMD` | desktop → robot | 3 doubles | `roll, pitch, yaw` (high-level + USER_POSE only) |
| `0x16 FULL_BODY_CMD` | desktop → robot | 140 doubles | `q[28] dq[28] u[28] kp[28] kd[28]` (user-ctrl mode only) |
| `0x18 COM_POS_CMD` | desktop → robot | 3 doubles | `x, y, z` (high-level + USER_POSE only) |
| `0x20 HEARTBEAT` | desktop → robot | empty | Server responds with `0xFE` |
| `0xFE ACK` | robot → desktop | 1 byte | `0x00` ok, `0x01` rejected |

## Licensing

This subdirectory is **GPL-3.0** because it imports AOS modules. The
surrounding `obelisk_westwood_cpp/` C++ code is MIT (Obelisk's license). See
[LICENSE](LICENSE).
