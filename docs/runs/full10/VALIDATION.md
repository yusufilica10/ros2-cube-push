# Latest autonomous mission validation — 2026-11-22

Result: **2 of 3 cubes delivered; full competition acceptance not passed.**

The autonomous headless Lyrical/Jetty run stopped at the 480-second wall-clock deadline (480.01 s recorded). Independent Gazebo pose evaluation confirmed that green and blue were fully inside their matching targets. Red remained near (0.705, -0.010) m and was not delivered. No manual intervention was used during this run.

Evidence: `full_run_mission.json` contains the mission events and deadline failure; `full_run_physical_check.json` contains independent physical containment results. The mission does not consume evaluator simulator poses.

Remaining work: reduce approach and realignment overhead and verify all three deliveries in a new autonomous run within 480 seconds. Current implementation must not be described as fully competition-ready.
