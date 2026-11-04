# Report data: WCR prototype digital twin

This document collects everything needed to write the report about the **digital twin of the physical prototype**: the hardware, the CAD model, the physics model, the simulation stack, the tests, the results and the lessons. It covers only this prototype twin; the earlier design-robot simulation (`wcr_dam_sim`, EDF and 4-wheel steering) is used only for its dam worlds.

The full test record, including every failed attempt, is in `docs/TEST_LOG.md`.

## 1. Goal

A simulation that behaves like **our physical robot**, with the same components, mass, geometry, thrust and drive. It should:

- predict where the hardware works and where it fails;
- run the inspection task (autonomous coverage of a dam face, finding cracks and wet patches) with the sensors the robot would need.

## 2. The physical prototype (input data)

| Part | Value used | Source |
|---|---|---|
| Chassis | 5 mm foam board, 290 × 200 mm, Ø170 mm hole under the prop, wheel notches at the corners | team, photo |
| Motor mount | plywood strip 200 × 25 × 4 mm across the hole (front–back), zip-tied | team |
| Thrust motor | A2212 1000 KV BLDC, 60 g with mount | team |
| Propeller | 1045 (10 × 4.5 in), 2 blades, Ø254 mm, blowing away from the wall; about 12 g (estimate) | team; mass estimated |
| ESC | 40 A, 24 g | team |
| Drive | 4 × N20 gear motor, 60 rpm, 10 g each, in standard white clamps (1.5 g each) | team |
| Wheels | Ø35 × 10 mm rubber tread, 20 g each ("heavy") | team |
| Motor driver | L298N, 25 g, at the front (+x) | team |
| Controller | ESP32 30-pin, 10 g, rear left | team, photo |
| Battery | 3S 11.1 V 2200 mAh, **off the robot** on an XT60 extension | team |
| Wiring | jumpers, 14/16 AWG, XT60: 38 g (the rest of the measured mass) | derived |
| **Total measured** | **370 g** | team (scale) |
| Thrust test (kitchen scale, servo-tester settings) | 2 → 100 g, 3 → 200, 4 → 250, 5 → 300, 6 → 350, 7 → 400 g | team |

Photos: `docs/figures/hw_prototype_photo.png` and `docs/figures/hw_n20_clamp.png`.

## 3. CAD model (`tools/build_proto_cad.py` → `cad/`)

The model was built in CadQuery from the rough measurements and the photo. Every dimension is a named constant at the top of the script.

**Coordinate frame.** The origin is the board centre on the wall-side face. +x is forward (the L298N end), +y is left, +z is away from the wall.

**Mass budget** (`cad/parts.json`):

| Group | g |
|---|---|
| foam board | 60 |
| plywood strip | 12 |
| BLDC motor + mount | 60 |
| propeller | 12 |
| ESC | 24 |
| 4 N20 motors | 40 |
| 4 clamps | 6 |
| 4 wheels | 80 |
| L298N | 25 |
| ESP32 | 10 |
| zip ties | 2 |
| wiring | 38 |
| **hardware total** | **369** (measured 370) |
| sensor kit (twin only) | 70 |
| **twin total** | **439** |

**Derived values:**

- Centre of mass (CAD frame): (−13.7, −1.9, 21.9) mm, which is **29 mm off the wall**.
- Board-to-wall gap: wheel radius 17.5 − axle height 10 = **7.5 mm**.
- Wheelbase 200 mm, track 190 mm.
- Prop hub 46 mm above the board face, which is 53.5 mm off the wall.

**Sensor kit (not on the hardware yet; needed for autonomy):**

| Sensor | Mass | Mounting |
|---|---|---|
| LD06-class 2D lidar | 42 g, plus a 10 g plywood mast | rear centre; scan plane 110 mm off the wall, above the prop disc |
| Wall camera | 6 g | front mast, 90 mm off the wall, tilted 70° toward the wall |
| Front camera | 6 g | same mast, looking ahead |
| MPU6050 IMU | 2 g | on the board |
| Wheel encoders | — | the encoder version of the N20 |

**Outputs:**

- `cad/wcr_proto.step` (opens in Fusion 360 or FreeCAD)
- `cad/preview.png`, `cad/preview_top.png`
- `cad/urdf_check.png`
- coloured meshes for Webots and ROS

## 4. Physics model and hand calculations

**Adhesion.** The propeller pushes air away from the wall, so its thrust T presses the robot onto the wall. In the simulation this is a force at the prop hub (53.5 mm off the wall) along −z of the robot. It follows the throttle setting with a 0.15 s motor lag.

On a face at angle β from horizontal, with friction coefficient μ and mass m:

```
normal force        N = T + m g cos β
grip available      F = μ N
weight along face   W = m g sin β
holds if  μ (T + m g cos β) ≥ m g sin β    →    T_needed = m g (sin β / μ − cos β)
peel / tip check    T · (wheelbase/2) ≥ m g sin β · h_CoM    (h_CoM = 29 mm: never the limit here)
```

**Thrust needed, μ = 0.7:**

| Face | β | 439 g (twin, with sensors) | 369 g (hardware only) | Measured max, 400 g | Full throttle, ~850 g (typical A2212 1000 KV + 1045 on 3S) |
|---|---|---|---|---|---|
| Winter dam | 53.5° | 243 g | 204 g | holds | holds |
| GoldenEye upstream | 71.8° | 458 g | 385 g | borderline | holds |
| GoldenEye gorge | 90.7° | 633 g | 532 g | **slides off** | holds |

The simulation reproduces this. The tyre grip was calibrated so that 400 g slides off the gorge and 850 g holds (TEST_LOG Phase F).

**Drive:**

- Top speed: v = ω r = (60 rpm × 2π / 60) × 0.0175 m = **0.11 m/s**.
- N20 rated torque is about 0.2 N·m. Torque is never the limit: friction is.
- Skid steer: left and right wheel speeds are v ∓ ω · (track × 1.5) / 2. The 1.5 factor accounts for the wheels scrubbing sideways, so the robot turns slower than the wheel speed difference suggests.

**Odometry.** Forward speed comes from the encoders, yaw rate from the gyro. The EKF fuses forward speed from the wheels with yaw rate from the IMU. The skid-steer yaw from the wheels is not used, because the wheels slip.

**Turning on a wall (the key result).** Webots uses a friction pyramid. A 4-wheel skid steer can only yaw if the sideways scrub resistance at ±wheelbase/2 is smaller than the wheel pull at ±track/2. On a wall, most of the pull is already used to hold the weight, so very little is left to steer with.

- With wheelbase 200 ≥ track 190, the robot **cannot turn on the spot** on a steep face. It slides down instead (winter dam: it could not turn back up from −49°).
- It **can** steer gently while driving. On the vertical gorge it manages about 5°/s at full thrust.

**Tyre model in Webots.** The final values are `coulombFriction [0.35 0.45]` (side, roll) and `forceDependentSlip [0.02 0]`.

- The asymmetric friction lets the skid steer turn.
- The sideways creep (FDS) lets it turn slowly instead of locking, like rubber.
- The asymmetric pyramid grips about 1.6× its coefficients. 0.35 / 0.45 was validated against the hand calculation (TEST_LOG Phase F).

## 5. Simulation stack

```
Windows: Webots R2025a  ── TCP 1234 ──  WSL Ubuntu, ROS 2 Humble (CycloneDDS)
  world _proto_<dam>.wbt                  webots_ros2_driver  → ProtoDriver plugin (thrust force, skid steer,
  robot WcrProto.proto                                         odometry, /joint_states, /ground_truth, /thrust)
                                          device topics       → /scan, /wall_camera, /front_camera, /imu
                                          robot_state_publisher (wcr_proto.urdf) → /tf, /robot_description
                                          wall_frames          world → map (the face) → odom
                                          robot_localization EKF   odom → base_link
                                          map_server (wall map) + Nav2 (Smac 2D planner, MPPI DiffDrive)
                                          coverage_mission (ProtoCoverage)   lanes → FollowPath / NavigateToPose
                                          inspection_detector  defects in the wall camera → /inspection/*
                                          foxglove_bridge  ws://localhost:8765 → Foxglove (Windows)
```

**Reused from `wcr_dam_sim` (unchanged):**

- the three dam worlds and their assets
- the inspection layouts (defects, obstacles, demo areas) and the wall maps
- `inspection_detector` and `wall_frames`
- the base coverage mission

**New for the twin:**

| Item | What it does |
|---|---|
| CAD | the prototype model |
| `build_proto_webots.py` | generates the PROTO and the URDF |
| `ProtoDriver` | thrust, skid steer, odometry |
| `ProtoCoverage` | coverage mission for skid steering: see below |
| Nav2 config | MPPI DiffDrive, footprint 0.32 × 0.26 m, lidar at 0.11 m |
| EKF config | gyro yaw |
| Foxglove layout | adds a thrust plot; view fixed to the face |
| Launch file | builds the world (tyre contact, box faces on the gorge and GoldenEye, fitted to the drawn faces); runs one Webots instance at a time |

**ProtoCoverage, changed for skid steering:**

- the robot always faces up the face: up-lanes forwards, down-lanes in reverse;
- lane changes are smooth S-curves with at most 20–30° of heading;
- lanes run 0.3 m past the area edge, so the S-curve happens outside the area;
- the goal tolerance is looser (0.15 m, 0.6 rad), because a skid steer can't correct a small sideways error at the goal.

## 6. Results (final: `results/final/`)

Autonomous coverage of each face's demo area (2.2 m × 2.0 m; 17 lanes, 12 cm apart, set by the 15 cm wall-camera footprint). Full throttle, 0.11 m/s.

| Face | Coverage | Defects found | Sim time | Path | Back at the start |
|---|---|---|---|---|---|
| Winter dam (53.5°) | 67.9 % | 2/2 (crack, wet patch) | 532 s | 39.2 m | yes |
| GoldenEye upstream (72°), live with Foxglove | 72.6 % | 2 | 405 s | 38.5 m | no |
| GoldenEye gorge (89°) | 73.4 % | 2 | 525 s | 40.6 m | no |

For comparison, the design robot (4-wheel steering, can crab sideways) reached about 85 % on the same areas.

Coverage maps: `results/final/coverage_<face>.png` (green = seen by the wall camera, grey = obstacle).

Hold tests at the measured 400 g:

| Face | Result |
|---|---|
| Winter dam | holds |
| GoldenEye upstream | holds (borderline) |
| Gorge | **slides off** |

At full throttle (850 g) it holds on all three faces.

## 7. Lessons for the hardware

1. **Thrust.** 400 g is enough for slopes up to about 60–70°, but **not for a vertical wall**. A vertical wall needs about 530 g without sensors and about 630 g with them. Propeller wall climbers usually need thrust of about 1.5–2× their weight on vertical walls.
2. **Wheel layout.** With wheelbase ≥ track, a 4WD skid steer can barely steer on a wall. Make the track wider than the wheelbase (≥ 1.2×), use 2WD plus castors, or add more thrust.
3. **Path planning.** A skid steer on a wall should not spin on the spot. Drive straight lanes, face up the face, and change lanes with gentle curves.
4. **Sensors.** Autonomy needs at least wheel encoders and an IMU, both cheap. Mapping and obstacle avoidance need a lidar, which adds weight (42 g or more) and so needs more thrust.
5. **Thrust control.** The ESC is set by a servo tester today. Letting the ESP32 set the throttle from the IMU tilt (`throttle:=auto`) would save battery on gentle faces.

## 8. Limitations and assumptions

- **Estimated values:** the dimensions are rough (taken from a photo); prop mass is assumed 12 g; full-throttle thrust is the typical datasheet value (850 g), not measured; N20 torque 0.2 N·m, μ 0.7 for rubber on concrete.
- **Not modelled:** propeller airflow details (suction through the hole, wall effects, wind), battery sag, the XT60 tether's weight and drag, motor heating, prop reaction torque.
- **Simulator approximations:** Webots friction is a pyramid. The tyre model is an approximation, calibrated only for holding and checked for turning.
- **Sensor kit:** not on the hardware yet. The twin's autonomy assumes it is added (+70 g).
- **Test area:** missions ran on the 4.4 m² demo area of each face, not the whole dam.

## 9. Reproduce

```
bash tools/wsl_build.sh                              # build the ROS package into ~/wcr_ws (needs wcr_dam_sim built there)
bash run_proto.sh goldeneye_gorge mission:=true      # live: Webots + Foxglove (import ros/wcr_proto/resource/wcr_proto_foxglove.json)
bash tools/wsl_all_missions.sh                       # the three missions (fast mode) → results/
bash tools/wsl_hold_test.sh goldeneye_gorge 7        # thrust hold test at the measured 400 g
bash tools/tests/arc_test.sh winter_dam full '0.35 0.45' '0.02 0'    # steering test
C:\Users\DEEPAK E L\.wcrvenv\Scripts\python.exe tools\build_proto_cad.py      # CAD → cad/, meshes
C:\Users\DEEPAK E L\.wcrvenv\Scripts\python.exe tools\build_proto_webots.py   # PROTO + URDF
```

## 10. Figures

| File | Shows |
|---|---|
| `cad/preview.png`, `cad/preview_top.png` | CAD model, isometric and top views |
| `cad/urdf_check.png` | URDF assembled (iso and side views) with the sensor frames |
| `docs/figures/hw_prototype_photo.png`, `hw_n20_clamp.png` | the real prototype and the N20 clamp |
| `docs/figures/webots_goldeneye_robot_on_fitted_face.png` | the robot in Webots on the GoldenEye upstream face |
| `docs/figures/foxglove_robot_model.png`, `foxglove_coverage_run.png`, `foxglove_live_goldeneye_old_layout.png` | Foxglove during the missions |
| `docs/figures/issue1_winter_stuck_turning.png` | stuck: cannot turn back uphill (TEST_LOG C1) |
| `docs/figures/issue2_mesh_no_asymmetric_friction.png` | Webots warning that led to the box faces (E1) |
| `docs/figures/issue3_gorge_fell_cameras_sky.png` | the robot fell off the gorge with the wrong tyre model (D2–D4) |
| `docs/figures/issue4_two_webots_instances.png` | two Webots windows, port 1235 (E2) |
| `docs/figures/issue6_foxglove_edge_on_view.png`, `issue7_foxglove_slanted_grid.png` | Foxglove camera in the world frame (H5) |
| `results/final/coverage_*.png` | coverage maps of the final missions |
