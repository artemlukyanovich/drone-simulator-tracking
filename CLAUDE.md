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
the detailed rationale.

## Current status

🚧 **Phase 1 done (2026-06-10) — ROS2 skeleton builds and runs.** All four `src/*` are
real ament_python packages. `drone_perception`/`drone_control` ship stub heartbeat nodes
(`detector_node`/`follower_node`); `drone_simulator` is a node-less skeleton (filled in
Phase 2); `drone_bringup` has `tracking_demo.launch.py`. `colcon build` is green and
`ros2 launch drone_bringup tracking_demo.launch.py` brings both nodes up. No simulator,
perception, or control logic yet — those land in Phases 2–3. The `/perception/target`
custom interface is deferred to Phase 3. Confirm capabilities against
`docs/project_plan.md` §8 before assuming a later-phase command works.

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

## First risk to close

PX4 / Gazebo / ROS2-distro versions are tightly coupled. Pin a working triple and record it
in `configs/simulator/` and `docs/project_plan.md` §11 **before** writing code.
