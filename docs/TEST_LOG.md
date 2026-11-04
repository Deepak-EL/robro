# Test log: WCR prototype digital twin

This log lists every simulation test run on the twin, in the order they were run (3–4 Nov 2026), with the settings used and the result. It includes the tests that failed and what was changed after each one, so it can be read as the trial-and-error record of the project.

**Where things are.** Scripts are in `tools/` and `tools/tests/`. The final reports are in `results/final/`, and screenshots are in `docs/figures/`.

**Common settings.** Unless a test says otherwise:

- Webots R2025a runs in fast mode, with ROS 2 Humble in WSL and CycloneDDS.
- The robot is 439 g: 369 g of hardware plus the 70 g sensor kit.
- `throttle:=full` means 850 g of thrust.
- Speed is 0.11 m/s, the N20 motors at 60 rpm.

Two terms used throughout:

- *Side* and *roll* friction are the tyre friction coefficients across the wheel and along its rolling direction, set as `coulombFriction [side roll]` in Webots.
- *FDS* is the force-dependent slip: tyre creep under load, in m/s per N.

---

## Phase A: first drive test (winter dam, 53.5°)

| # | Test | Settings | Result |
|---|---|---|---|
| A1 | Hold on the wall after the deployment ledge is removed | μ 0.7 in both directions, no FDS | 0.000 m slide: **holds** |
| A2 | Forward 8 s at 0.11 m/s | same | 0.878 m, matching 60 rpm × π × 35 mm; heading stayed 0° |
| A3 | Turn on the spot, 6 s at 0.5 rad/s | same | **turned only 17°, slid 0.54 m down the face** |
| A4 | Idle after turning | same | slid a further 0.58 m |

Script: `tools/tests/drive_test.sh`.
Conclusion: the robot holds and drives straight, but cannot turn on the spot on the slope.

## Phase B: why it can't turn (winter dam, arc tests)

Script: `tools/tests/arc_test.sh WORLD THROTTLE 'side roll' 'fds'`.
The test drives arcs forwards and backwards (wz ±0.2 / ±0.5 / ±0.3 rad/s), then spins on the spot, and measures yaw and slide from the true pose.

| # | Tyre setting | Yaw on every arc and spin | Interpretation |
|---|---|---|---|
| B1 | μ 0.7 (symmetric) | **exactly 0°** (locked) | Webots (ODE) uses a friction *pyramid*. With wheelbase 200 mm ≥ track 190 mm, sideways scrub always beats the wheel pull, so it cannot yaw. |
| B2 | 0.7, FDS [0 0.02] | 0° | no change |
| B3 | [0.7 0.5] | 0°, numbers identical to B1 | the first value is *not* the rolling direction |
| B4 | [0.7 0.3] | 0° | still locked |
| B5 | **[0.3 0.7]** | **+97° in 6 s**, turns both ways | the first value (fdir1) is the wheel's **sideways** direction |
| B6 | [0.55 0.7] | turns; slides 0.27 m per spin; turning "downhill" is easier than "uphill" | this became the default |

Turning is only possible when side < roll × track / wheelbase, which is 0.665 × roll.

## Phase C: autonomous mission, first attempts (winter dam)

| # | Change | Result |
|---|---|---|
| C1 | Nav2 MPPI DiffDrive inside a RotationShim, which turns on the spot toward each leg; the robot faces its direction of travel | **stuck.** At −49° yaw, wz +0.15…+0.5 commanded, zero rotation. It cannot turn back uphill. See `docs/figures/issue1_winter_stuck_turning.png`. |
| C2 | Robot always faces up the face (up-lanes forwards, down-lanes in reverse); lane changes as S-curves; no RotationShim | it drove the first S-curve to −28°, then **could not steer back** (wz +0.23 commanded, no change) |
| C3 | Added sideways tyre creep, FDS [0.02 0] | **mission completed: 63.1 % coverage**, 2 defects, back at the start. 16 of 17 lanes ended "not completed": the robot stalls trying to square up at the exact end pose. |

## Phase D: three faces, tuning

| # | Face | Settings | Coverage | Notes |
|---|---|---|---|---|
| D1 | GoldenEye upstream (72°) | [0.55 0.7]; goal tolerance 0.08 m / 0.6 rad; lanes run 0.3 m past the area edge | 73.2 % | 3 defects; home leg not reached |
| D2 | Gorge (89°), arc test | [0.55 0.7] + FDS | almost no steering (2° per 6 s); a spin on the spot → **fell off the wall** | see the Phase E cause |
| D3 | Gorge, arc test at 1200 g | same | steers about 4°/s | |
| D4 | Gorge mission at 1200 g | same | 14.7 % | robot slid to the bottom of the face during the first lane change |
| D5 | Gorge mission at 2000 g | same | **invalid** | a second Webots window was open; see Phase E |
| D6 | Winter dam | goal tolerance 0.15 m, overshoot 0.3 m | 73.6 % | |

## Phase E: simulation problems found and fixed

| # | Symptom | Cause | Fix |
|---|---|---|---|
| E1 | Webots console: "A Mesh is used in a Bounding object using an asymmetric friction" (`issue2_mesh_no_asymmetric_friction.png`) | The gorge face collision is a **mesh**, and Webots ignores asymmetric friction on meshes. So D2–D5 had the wrong tyre model. | Measured the gorge collision mesh: it is exactly planar over the drive area (0.0 mm deviation, `tools/tests/visual_vs_face.py`). The launch adds a flat box face. |
| E2 | Two Webots windows open (`issue4_two_webots_instances.png`) | Stopping the ROS launch doesn't close the Windows program. The next run took port 1235, and its controller drove the **old** window. | The launch refuses to start if Webots is open, and closes Webots on shutdown. The test scripts also close it. |
| E3 | Gorge on the box face, arc test at 850 g, [0.55 0.7] | — | It now steers about 90° per 6 s and holds. So the D2/D4 failures were the mesh problem, not the physics. |
| E4 | Gorge mission at 850 g on the box face | — | **73.1 %**, 2 defects |

## Phase F: tyre grip calibration

Script: `tools/wsl_hold_test.sh WORLD THROTTLE [side roll] [fds]`.
The test spawns the robot on the ledge, waits for the ledge to be removed, drives 0.2 m up, parks, and measures the slide over 20 s.

Hand calculation: the robot holds if μ (T + m g cos β) ≥ m g sin β, i.e. if T ≥ m g (sin β / μ − cos β) (see REPORT_DATA §4). For the vertical gorge with μ 0.7 and 439 g, that is T ≥ **633 g**.

| # | Tyre | Gorge 300 g | Gorge 400 g | Gorge 850 g | Winter 400 g | GoldenEye 400 g | Comment |
|---|---|---|---|---|---|---|---|
| F1 | [0.55 0.7] | slides | **holds** | holds | holds (even 100 g holds!) | holds | grips too hard |
| F2 | 0.7 symmetric | — | slides | — | — | — | matches the hand calculation |
| F3 | **[0.35 0.45] + FDS [0.02 0]** | — | **slides** | **holds** | holds (needs 243 g) | holds (needs 458 g: borderline) | **final default.** Asymmetric pyramid friction grips about 1.6× its coefficients. |

Arc test on the gorge with F3: it steers about 5°/s at full thrust and holds when parked.

Two bugs in the hold test itself were found and fixed:

1. At 0 g the robot fell before the test started measuring at 6 s, so it reported "holds". The test now measures from the spawn and drives off the ledge first.
2. The first version measured while the robot was still resting on the ledge.

## Phase G: final missions (F3 tyres, demo area 2.2 × 2 m)

| Face | Coverage | Defects found | Sim time | Path | Lanes "not completed" | Home reached |
|---|---|---|---|---|---|---|
| Winter dam, 53.5° | **67.9 %** | crack_0, wet_patch_0 | 532 s | 39.2 m | 11/17 | yes |
| GoldenEye upstream, 72° (live, real time, Foxglove) | **72.6 %** | crack_0, wet_patch_0 | 405 s | 38.5 m | 2/17 | no (stopped short) |
| GoldenEye gorge, 89° | **73.4 %** | crack_0, wet_patch_0 | 525 s | 40.6 m | 9/17 | no (stopped short) |

The reports and coverage maps are in `results/final/`. The bush was in the costmap and avoided on every face.

## Phase H: visual checks

| # | Check | Result |
|---|---|---|
| H1 | URDF assembly, drawn the way RViz and Foxglove assemble it (`tools/render_urdf.py`) | correct: 25 visuals, wheels on the wall plane, wall camera pitched 70°, lidar above the prop (`cad/urdf_check.png`) |
| H2 | Robot drawn inside the wall on GoldenEye upstream | The drawn face mesh (obj_Object_7) is 7–24 mm **behind** the dam sim's collision box and tilted 0.2°. A lower band (obj_Object_11) is 28–33 mm **in front**. Fix: fitted a plane to the drawn face (`tools/fit_face.py`: h = −23.8 mm + 3.58 mm/m·s + 0.23 mm/m·t, rms 2.9 mm), moved the old box back 5 cm, pushed the band behind. `docs/figures/webots_goldeneye_robot_on_fitted_face.png` |
| H3 | Foxglove can load the robot model | 19/19 mesh assets served by foxglove_bridge (`tools/tests/foxglove_asset_check.py`) |
| H4 | Propeller still in Foxglove | it was a fixed joint on purpose (a 900 rad/s rotor only flickers). Now a continuous joint with a 12 rad/s display angle (`tools/tests/prop_check.sh`) |
| H5 | Foxglove 3D view slanted / edge-on (`issue6_foxglove_edge_on_view.png`, `issue7_foxglove_slanted_grid.png`) | The camera was in the world root frame, and the face is steep or vertical. Fix: follow mode Pose on the `map` frame (the face), 2D view. |
