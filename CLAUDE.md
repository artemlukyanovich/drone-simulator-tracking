# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Vision-guided drone control in simulation: a PX4 drone tracks and follows a target it
detects through its own camera. This is the **control** half of the `ai_drones` ecosystem;
the **perception** half is the separate repo `project_1/real-time-object-counter`. The flow
is `Camera → Detector → Tracking → Decision → Move`.

**Source of truth for decisions, architecture, and the phased plan is
[`docs/project_plan.md`](docs/project_plan.md) (written in Russian). Read it before making
non-trivial changes.** This CLAUDE.md is a quick operational summary; `project_plan.md` is
the detailed rationale. For a full architecture overview (per-module breakdown, end-to-end
data/action flow, domain concepts) see [`docs/architecture.md`](docs/architecture.md).

## Current status

✅ **Phase 3 complete — MVP "see → move" closed (2026-07-06). Detail & source of truth:
[`docs/phase3_setup.md`](docs/phase3_setup.md).** (Phase 2 done: PX4 **v1.15.4** SITL
`gz_x500_mono_cam` flies in Gazebo Harmonic, uXRCE-DDS bridge exposes `/fmu/*` at 100 Hz via
`px4_msgs` `release/1.15`, camera → `/camera/image` ~30 Hz.) **Next: Phase 4 — plan agreed
(2026-07-07), not yet implemented.** Reframed as "closer to real conditions" (multi-target
track+lock, search/reacquire FSM, gimbal + honest distance, PID, safety failsafes, telemetry
dashboard). Detail & source of truth: [`docs/phase4_setup.md`](docs/phase4_setup.md); summary in
`docs/project_plan.md` §8.

**Increments 0–4A done (✅)** — full pipeline flies against a static target:
- `drone_interfaces/Target.msg` — custom interface (`detected`, normalized `offset_x/y`, `area_ratio`).
- Own world `src/drone_simulator/worlds/follow_target.sdf` (Fuel "Standing person"), launched
  **standalone**: `WORLD=… scripts/run_px4_sitl.sh` (PX4 attaches via `PX4_GZ_STANDALONE`;
  `<world>` must be named `default`).
- `detector_node` — YOLO over `/camera/image` → `/perception/target` (+ `/perception/image`).
- `follower_node` — offboard loop + P-controller, **validated with a fake target** (arms,
  takes off, reacts correctly).
- `tracking_demo.launch.py` — real pipeline (`follower_node`+`detector_node`+configs, with
  `detector_delay_s`/`detector_device` diag knobs). **Gate 4A passed (2026-07-05):** drone
  arms, takes off, turns to the person, holds distance (`offset_x→0`, `area_ratio≈0.15`).

**Increment 4B done (✅) — Gate of Phase 3 (2026-07-06, §15):** static `Standing person`
replaced by a walking Gazebo `<actor>` (`Mingfei/actor` `walk.dae`) on a **forward-offset
circle** (center `(8,0)`, R=3) — the drone detects the walking person and **physically flies
after it** across the scene. Two gotchas learned & handled: (1) target path must be **offset
from the drone's spawn** — an orbit centered on the drone only needs yaw (drone spins in
place); (2) horizontal camera at 2.5 m can't hold a ground target closer than ~3 m (it drops
off the frame bottom), so `area_target` was retuned `0.15→0.02` with `kp_forward 3→18`,
`area_deadband 0.02→0.004`. Also: start `detector_node` **after** camera warm-up
(`tracking_demo.launch.py detector_delay_s:=8`) to dodge the startup render race. Only
`follow_target.sdf` + `configs/control/follower.yaml` changed; node code untouched.

⚠️ **Hybrid-graphics gotcha (was the "won't arm" blocker, now resolved):** run the sim with
**`GPU=nvidia`** on this laptop. Without it, YOLO on the integrated GPU contends with Gazebo's
render → lockstep stalls → PX4 loses sensors (`Compass Sensor 0 missing`) *and* the camera
degrades → arming denied. `GPU=nvidia` fixes both. See `docs/phase3_setup.md` §9.

⚠️ **Build with `scripts/build.sh`, not bare `colcon build`** — nodes with pip deps
(`torch`/`ultralytics`) need the venv-python shebang (see `docs/phase3_setup.md` §12).

PX4-Autopilot and px4_msgs are external code (in `~/src/` and `src/px4_msgs/`, both
git-ignored, fetched per `docs/phase2_setup.md`). Confirm capabilities against
`docs/phase3_setup.md` and `docs/project_plan.md` §8 before assuming a command works.

## Key decisions (see project_plan.md §3 for rationale)

- **ROS2-first**, not MAVSDK-first. Native PX4 link is **uXRCE-DDS + `px4_msgs`**; MAVSDK
  is optional/later.
- **Multi-repo**: this is its own git repo, independent from `project_1`. The parent
  `ai_drones/` is just a filesystem container, not a repo.
- Perception from `project_1` is **ported as logic into ROS2 nodes**, not pip-installed and
  not shared as a package. Divergence from `project_1` is expected and acceptable.
- Separate per-project virtual environment.

## This is a colcon workspace

Not a flat Python project. Code lives in **ament packages** under `src/`, is built with
`colcon build`, and run via `ros2 run` / `ros2 launch` — not `python main.py`.

```
src/drone_simulator   SITL launch, bridges, world/drone description
src/drone_perception  detector_node (YOLO over /camera/image)
src/drone_control     follower_node (offboard loop, PID)
src/drone_bringup     launch files that wire nodes together
```

Build/run flow (once packages exist):
```bash
colcon build
source install/setup.bash
ros2 launch drone_bringup tracking_demo.launch.py
```

## Conventions and hard rules

- **Package/node names** are `snake_case`, no hyphens (hyphen only in the repo/workspace
  name). ament_python requires the double nesting `src/<pkg>/<pkg>/code.py` — this is
  correct, not a mistake.
- **Nodes communicate via topics/services, never by importing each other.** The pipeline is
  assembled in launch files in `drone_bringup`, not hardcoded.
- **PX4 offboard requires a continuous setpoint stream at >2 Hz** or it drops out of
  offboard mode. The controller is a *loop* publishing setpoints every tick (even "hover" =
  zero velocity), not a one-shot command. Stream setpoints *before* switching mode, then arm.
- Be explicit about coordinate frame (body vs NED) when mapping bbox offset → setpoint.
- **Never commit** `build/`, `install/`, `log/`, `outputs/` contents, model weights
  (`*.pt`/`*.onnx`/`*.engine`), venvs, or `.idea/`.
- Configuration goes in `configs/`, separate from code. No magic numbers in code.
- Docs are written in Russian; code and identifiers in English.

## Architecture (target)

```
Gazebo (physics + camera) → ros_gz_bridge → /camera/image
  → drone_perception:detector_node → /perception/target (bbox + offset from frame center)
  → drone_control:follower_node → OffboardControlMode + TrajectorySetpoint (via uXRCE-DDS)
  → PX4 SITL → motors in Gazebo
```

## First risk (closed in Phase 2)

PX4 / Gazebo / ROS2-distro versions are tightly coupled. The working stack is pinned and
verified: ROS2 **Humble** + Gazebo **Harmonic 8.13** + PX4 **v1.15.4** + px4_msgs
`release/1.15` + Micro-XRCE-DDS-Agent v2.4.3 — recorded in `configs/simulator/stack.md` and
`docs/project_plan.md` §11, with full reproduction steps in `docs/phase2_setup.md`.
