# Dumb-E

5-DOF 3D-printed arm (20:1 cycloidal joints: base yaw, shoulder, elbow, forearm roll, wrist pitch; geometry
from the author's CAD, see `docs/kinematics.md`) + servo gripper. ESP32 DOIT DevKit V1 (WROOM-32) on PCB `test-dumbev2`,
TMC2209 standalone (STEP/DIR/EN only). Firmware in C, **ESP-IDF v6.0.3**, target `esp32`.

## Layout
- `firmware/` ESP-IDF project; `components/`: `kinematics` `motion` `stepgen` (pure-C parts host-tested),
  `robot` (state machine + `robot_config.c` = mechanical params), `board` (only place with GPIO numbers),
  `gripper`, `comms` (serial protocol), `net`.
- `firmware/test/host/` Unity tests; `firmware/test/qemu/` end-to-end smoke test.
- `docs/`: architecture, protocol, kinematics, bringup, roadmap. `hardware/pcb/REVIEW.md`: PCB pinout + issues.

## Commands
```sh
cmake -S firmware/test/host -B build-host && cmake --build build-host && ctest --test-dir build-host
cd firmware && idf.py build
# QEMU (from firmware/): see header of test/qemu/smoke_test.py
```
CI (`.github/workflows/ci.yml`) runs host tests + firmware build.

## Rules
- Code, comments, commits in English; docs in Italian.
- Commits: author Paolo Vezzini <megampaul@icloud.com>; no Co-Authored-By / Claude-Session trailers.
- Pure-C files (`kinematics.c`, `motion_profile.c`, `stepgen_dda.c`, `segment_ring.h`) must not include
  ESP-IDF headers. Units inside: mm, rad, s; degrees only in `comms`.
- Step ISR (`stepgen.c`): integer only, IRAM, no flash calls (`CONFIG_GPTIMER_ISR_CACHE_SAFE=y`). Keep it that way.
- Our components build with `-Wall -Wextra -Werror`; APIs return `esp_err_t` / `robot_err_t`.
- Protocol responses: one line `ok ...` / `err <code> <msg>`, written in a single write (`out()` in comms.c).
  Update `docs/protocol.md` when commands change.
- No NVS/flash writes while moving.
- CAD files are not in the repo (cycloidals derived from purchased SweepDynamics models): never commit them.
- Pins only in `components/board`; mechanical values only in `robot_config.c` (currently placeholders).
- Details: `docs/architecture.md`, `docs/roadmap.md` (next phases).
