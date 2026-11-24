# Autonomous mission validation — 2026-11-24

**PASS: 3/3 matching targets, 464.45 seconds wall time (7 min 44.45 s).**

One full autonomous run (`full14`) completed without manual intervention after
its mission timer started. Preparation is excluded; the unchanged deadline is
480 seconds wall time. Remaining margin in this run: 15.55 seconds.

## Independent physical check

`full_run_mission.json` records `COMPLETE`, green/blue/red delivered and no
failures. `full_run_physical_check.json` independently checks all eight rotated
corners of each 0.18 m cube against its matching 0.60 m target. It confirms:

| Cube | Final world center (m) | Fully inside target |
|---|---|---|
| green | (2.4017, 0.0527) | Yes |
| blue | (2.3290, -1.1832) | Yes |
| red | (2.3452, 1.1765) | Yes |

The evaluator consumes Gazebo poses only after the autonomous test; mission
control consumes camera detections, TF and odometry, never evaluator poses.
The archived log and observer output are in `docs/runs/full14/`.

## Conditions and checks

- Ubuntu 26.04, ROS 2 Lyrical, Gazebo Jetty 10.5; 4 virtual CPUs, software rendering.
- Headless simulation, RViz off; GUI performance was not timed in this run.
- Original SLAM map `arena_check_20261122_153109.yaml`; fixed competition arena.
- Order green, blue, red; outer cubes go directly to the left-side center line.
- Normal push speed 0.25 m/s; slower approach and camera-relative final placement.
- Fast DDS UDPv4, asynchronous publication; domain 47.
- Physics timestep remains 2 ms. Joint states publish at 20 Hz; unmodified
  Gazebo time stamps are distributed to ROS at up to 50 Hz by `clock_throttle`.
- Package build passed; colcon reports 11 tests, zero errors/failures.
- Clock relay integration check passed: rate limit, exact stamps, backward reset.
- SDF validation and URDF-to-SDF conversion passed (sensor extension warnings).

This is evidence of one passing full run, not a repeatability measurement.
The previous full10 run delivered 2/3 before the deadline; its evidence is
preserved under `docs/runs/full10/`. Full11/13 were communication/data-loss
failures; full12 was interrupted for performance work and is not a full result.

## Reproduce

From the workspace root, terminal 1:

```bash
source ./env.sh
LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless ros2 launch cube_push_sim competition.launch.py headless:=true rviz:=false
```

Terminal 2, after sourcing the same `env.sh`:

```bash
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/status --field data --full-length
```

After `COMPLETE`, run the independent evaluator while the simulator is still on:

```bash
ros2 run cube_push_sim evaluate_run --output /tmp/physical_delivery_check.json
```

Reset the entire simulation between attempts. Do not use teleop, RViz goals or
Gazebo pose editing during a timed autonomous run.
