# CarMaker 15–Autoware Integration Session Process Journal

Date completed: 2026-07-29  
Status: working end-to-end, as confirmed by the user  
Supported TestRun: `FS_autonomous_Sprint_LTS`

## 1. Objective and scope

The objective was to integrate CarMaker 15.0 with Autoware so that:

1. CarMaker supplies the simulated vehicle pose and sensor data.
2. Autoware localizes the vehicle, plans a route and trajectory, and produces control commands.
3. The final Autoware control command is applied to the CarMaker vehicle.
4. Autoware and RViz can remain running when a CarMaker TestRun stops and starts again.

The work was deliberately isolated:

| Purpose | Path |
|---|---|
| Original Autoware checkout, not modified for this integration | `/home/klee/autoware` |
| Isolated Autoware workspace used by the integration | `/home/klee/autoware.testvehicle` |
| Original CarMaker 15 project, preserved | `/home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15` |
| Duplicated CarMaker project used by the integration | `/home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware` |

This separation was the first design decision. All integration packages, maps,
launch files, and Autoware-side changes were placed in the isolated workspace.
All CarMaker source and vehicle changes were placed in the duplicated project.

## 2. Final architecture

The completed signal path is:

```text
CarMaker 15
  ├─ /clock
  ├─ /carmaker/odometry
  ├─ /carmaker/lidar/front/points
  ├─ /carmaker/imu
  └─ /carmaker/steering_angle
            │
            ▼
autoware_carmaker_interface
  ├─ ground-truth localization + map→base_link TF
  ├─ /initialpose3d and localization initialization state
  ├─ PointXYZIRC LiDAR input for Autoware perception
  ├─ vehicle velocity, steering, gear, control-mode and actuation status
  └─ restart detection, route clearing and command watchdog
            │
            ▼
Autoware perception → planning → trajectory follower
            │
            ▼
/control/command/control_cmd
  → raw_vehicle_cmd_converter
  → /control/command/actuation_cmd
  → autoware_carmaker_interface
  → /carmaker/VehicleControl
  → CMNode VehicleControl callback
  → CarMaker gas, brake, selector and road-wheel steering
```

The seven isolated ROS packages are:

- `vehiclecontrol_msgs`
- `autoware_carmaker_interface`
- `carmaker_fsai_vehicle_description`
- `carmaker_fsai_vehicle_launch`
- `carmaker_sensor_kit_description`
- `carmaker_sensor_kit_launch`
- `autoware_carmaker_launch`

The final launch intentionally uses CarMaker ground-truth localization, so the
normal Autoware sensing drivers and localization stack are not started.
Planning, LiDAR perception, control, system, and AD API nodes are started.

## 3. Chronological implementation and debugging record

### Phase 1 — Establish isolated workspaces

1. The main Autoware checkout was inspected but was not used as the integration
   target.
2. `/home/klee/autoware.testvehicle` was selected as the only Autoware runtime
   workspace.
3. The CarMaker project was replicated as
   `FSAI_LTS_2026_CM15_Autoware`.
4. The isolated Autoware manifest was aligned with the test-vehicle launcher
   fork used by this vehicle configuration.
5. Builds were forced to use `/usr/bin` and ROS Humble's system Python. This
   avoided a Conda Python taking precedence over the Python 3.10 environment
   used by ROS Humble.

### Phase 2 — Port the CarMaker ROS interface to the required contracts

The initial CMROSIF example was not a complete Autoware vehicle interface. It
contained older example assumptions and a legacy point-cloud contract.

The CarMaker-side work therefore included:

1. Building against the duplicated CarMaker 15 project.
2. Updating the CMNode version header for CarMaker 15.
3. Adding the shared `vehiclecontrol_msgs/VehicleControl` message to both ROS
   workspaces.
4. Adding a `/carmaker/VehicleControl` subscriber.
5. Scheduling that subscriber at the `VehicleControl` callback hook.
6. Applying the received selector, gas, brake, steering angle, steering
   velocity, and steering acceleration after CarMaker's normal
   `VehicleControl_Calc()` stage through `User_VehicleControl_Calc()`.
7. Adding CarMaker odometry, IMU, steering feedback, and LiDAR publishers.

An early generic `carmaker_autoware_bridge` could relay sensors and discover a
control message at runtime, but it did not provide the complete localization,
vehicle status, reset, diagnostic, and exact actuation contracts required by
Autoware. The operational design was consequently replaced by the dedicated
packages in `src/carmaker_integration`.

### Phase 3 — Implement the Autoware vehicle interface

`autoware_carmaker_interface` was created to convert CarMaker output into the
full set of Autoware inputs:

- `/localization/kinematic_state`
- `/localization/initialization_state`
- `/initialpose3d`
- `/localization/pose_estimator/pose_with_covariance`
- `/localization/acceleration`
- `/sensing/lidar/concatenated/pointcloud`
- `/sensing/imu/carmaker/imu_raw`
- `/vehicle/status/velocity_status`
- `/vehicle/status/steering_status`
- `/vehicle/status/gear_status`
- `/vehicle/status/control_mode`
- turn-indicator, hazard-light, and actuation status

It also converts Autoware actuation and gear commands into
`/carmaker/VehicleControl`.

A wall-clock watchdog was added. If a command has been active and then becomes
older than 0.5 seconds, the interface commands zero gas and 35% brake. The
watchdog and diagnostics use a system clock so they still operate if simulated
time is paused or stopped.

### Phase 4 — Fix the traffic-light component crash

Observed failure:

```text
process has died ... component_container_mt
__node:=traffic_light_node_container
__ns:=/perception/traffic_light_recognition/camera7
exit code -6
```

Diagnosis:

The default Autoware launch attempted to create traffic-light recognition
pipelines for camera namespaces such as `camera7`. The Sprint sensor kit did
not provide the expected traffic-light camera inputs. This unused composable
pipeline could abort its container.

Fix:

- Set `use_traffic_light_recognition=false`.
- Set `traffic_light_recognition/use_high_accuracy_detection=false`.
- Retained LiDAR perception.

Result:

The unused camera6/camera7 traffic-light path no longer starts and the
traffic-light container abort is removed.

### Phase 5 — Fix the LiDAR point layout

Observed failure:

```text
The pointcloud layout is compatible with PointXYZI.
You may be using legacy code/data
```

Diagnosis:

The example publisher used the old `sensor_msgs/PointCloud`/PointXYZI-style
layout. The active Autoware version expects its LiDAR input in PointXYZIRC
form. A syntactically valid `PointCloud2` is not sufficient if the fields,
offsets, types, or point step do not match this contract.

Fix:

The CMNode LiDAR publisher was changed to `sensor_msgs/msg/PointCloud2` with a
packed 16-byte PointXYZIRC layout:

| Field | Type | Offset |
|---|---|---:|
| `x` | float32 | 0 |
| `y` | float32 | 4 |
| `z` | float32 | 8 |
| `intensity` | uint8 | 12 |
| `return_type` | uint8 | 13 |
| `channel` | uint16 | 14 |

The topic is `/carmaker/lidar/front/points` and the frame is
`carmaker_lidar`.

Result:

Autoware accepted the cloud layout. A later “compatible callback” warning from
CenterPoint was a performance-path warning, not the legacy layout error.

### Phase 6 — Resolve localization readiness, MRM toggling, and stale commands

Observed failures:

```text
MRM State changed: NORMAL -> MRM_OPERATING
EMERGENCY_STOP is operated.
vehicle_cmd_gate: Emergency!
Autoware actuation command stale for 0.520s; applying safe brake
/initialpose3d has not received
/autoware/localization/state STALE
/autoware/planning/routing/state STALE
route ERROR
trajectory ERROR
trajectory_follower ERROR
```

Diagnosis:

These messages were not all independent defects:

1. Autoware did not yet have a complete localization readiness contract.
2. Without a route, planning produced no trajectory.
3. Without a trajectory, the controller produced no actuation command.
4. The interface then correctly identified the old command as stale and
   applied the safe brake.
5. The diagnostic graph rejected autonomous mode and the MRM handler requested
   an emergency stop.

The stale command was therefore mainly a downstream safety symptom, not the
first cause.

Fix:

- Publish ground-truth kinematic state and `map → base_link`.
- Publish localization initialization state with transient-local durability.
- Publish the first valid CarMaker pose on `/initialpose3d`, also
  transient-local.
- Publish the complete vehicle status expected by planning and control.
- Add raw vehicle command conversion and actuation status.
- Add a CarMaker-specific diagnostic graph with simulator-appropriate startup
  timeouts. Missing, stale, or invalid route/trajectory/localization data still
  blocks autonomous mode.
- Treat `WaitingForRoute` as the expected pre-route state, not as a vehicle
  interface failure.

Result:

Localization reaches the initialized state, Autoware reaches
`WaitingForRoute`, and autonomous availability becomes healthy after a route
and a live trajectory exist.

### Phase 7 — Fix the initial empty/invalid occupancy transform

Observed failure:

```text
probabilistic_occupancy_grid_map:
Failed to lookup transform: Invalid argument "" passed to lookupTransform
argument source_frame - in tf2 frame_ids cannot be empty
```

Diagnosis:

A sensor message could enter perception before a valid localization/TF epoch
was available. Closely related timestamp skew was also found between the first
LiDAR sample and the first localization transform.

Fix:

LiDAR and IMU forwarding was gated until a matching odometry-based
localization stamp existed. Empty or old-epoch sensor data is no longer sent
into the Autoware pipeline.

### Phase 8 — Make repeated CarMaker TestRuns safe

Observed behavior:

After stopping and restarting the CarMaker simulation, the Autoware map/vehicle
view was no longer usable without restarting Autoware.

The interface later logged the reset clearly:

```text
Detected a new CarMaker TestRun (58.290s -> 0.010s).
Reinitializing localization and applying safe brake.
```

Diagnosis:

`SimCore.Time` returned to zero for each TestRun. Publishing this directly on
`/clock` made ROS simulated time jump backward. TF caches, RViz message
filters, timers, and old routes then contained data from an incompatible time
epoch.

Fix, CarMaker side:

- Added a cumulative clock epoch offset to CMROSIF.
- Tracked the last raw and last published clock values.
- When raw simulation time regresses, advance the offset so `/clock` remains
  monotonic across TestRuns.

Fix, Autoware interface side:

- Detect raw odometry time regression with a 0.05-second tolerance.
- Immediately apply safe brake and hold normal commands.
- reset localization state and republish the new ground-truth initial pose.
- clear the route from the previous TestRun through
  `/api/routing/clear_route`.
- retry route clearing if the service is temporarily unavailable or rejects
  the first request.
- release the command hold only after reset handling is complete.

Result:

Autoware and RViz can remain running. A restarted TestRun returns to
`WaitingForRoute`; the operator intentionally selects a new goal for the new
run.

### Phase 9 — Fix the first LiDAR sample after restart

Observed failures:

```text
Lookup would require extrapolation into the past.
Requested time 0.010000 but the earliest data is at time 0.012000,
when looking up transform from frame [map] to frame [carmaker_lidar]

Message Filter dropping message: frame 'carmaker_lidar' at time 0.010
for reason 'the timestamp on the message is earlier than all the data
in the transform cache'
```

Diagnosis:

The LiDAR publisher could emit a sample at 0.010 seconds while the first valid
odometry-derived transform appeared at 0.012 seconds. On a restart, a sensor
could also reset before the odometry callback had detected the new TestRun.
That allowed a new-epoch cloud to be compared with old-epoch localization.

Fix:

- Track the raw timestamps of odometry, LiDAR, and IMU independently.
- Detect when a sensor timestamp regresses before odometry does.
- Invalidate old localization for that sensor epoch.
- Drop/hold LiDAR and IMU until odometry establishes the corresponding new
  localization epoch.

Result:

The first cloud no longer precedes the TF cache and perception starts from a
coherent timestamp epoch.

### Phase 10 — Find why an accepted route still produced no motion

The logs confirmed that the goal was valid and the route was accepted:

```text
Route set via set_waypoint_route
Initial pose - x: 256.102544, y: -120.416743
Goal pose - x: 234.925613, y: -132.263168
AutowareState: WaitingForRoute => Planning
```

However, downstream nodes reported:

```text
behavior_path_planner: waiting for occupancy_grid
controller_node_exe: Waiting for trajectory data
Control is skipped since input data is not ready.
/control/trajectory_follower/control_cmd has not received
```

Diagnosis:

The default pointcloud-based occupancy implementation synchronizes the latest
raw and segmented clouds using exact timestamp equality. CarMaker's ROS job
runs every 10 ms, while the LiDAR data updates every 20 ms. A scan could be
repeated and retimestamped, while the segmented output lagged. The two “latest”
timestamps then never became exactly equal. No occupancy grid meant no behavior
path, no trajectory, no controller output, and no autonomous mode.

Fix:

The wrapper now uses Autoware's simulator-style occupancy configuration:

```text
occupancy_grid_map_method=laserscan_based
input_obstacle_pointcloud=true
input_obstacle_and_raw_pointcloud=false
```

This consumes the segmented obstacle cloud and does not require exact pairing
with the raw cloud.

Result:

`LaserscanBasedOccupancyGridMapNode` loads, the occupancy grid is published,
and behavior planning can progress.

### Phase 11 — Correct the generated map centerline

After occupancy was repaired, StartPlanner still produced repeated pull-out
failures:

```text
ClothoidPullOut: Circular path generation failed ...
Relative pose info: -0.483000, -2.000000, 0.000000
```

Diagnosis:

The Sprint map generator initially used a 2.0 m lane-center offset. The actual
CarMaker FSAI start pose is on the IPGRoad reference line at offset 0.0. The
generated lanelet therefore made the vehicle appear approximately 2 m
laterally displaced from the lane. StartPlanner could not find a safe pull-out
and generated a stop path.

Fix:

- Changed the generator's default `--lane-center-offset` from 2.0 to 0.0.
- Regenerated the Sprint Lanelet2 map and point-cloud map.
- Checked the generated center against the actual start pose.

Validation:

At 1 m map sampling, start pose
`(256.102544, -120.416743)` is approximately 0.124 m from the generated
centerline instead of 2 m away.

### Phase 12 — Correct the CarMaker drive-torque interpretation

Even after a moving trajectory and positive Autoware acceleration command were
available, the vehicle could remain at the start.

Diagnosis:

The duplicated vehicle initially contained:

```text
PowerTrain.Control.GasInterpret.TrqFull = 55
PowerTrain.Control.GasInterpret.TrqZero = -55
```

With linear interpretation, this is approximately:

```text
T = (55 - (-55)) × gas - 55 = 110 × gas - 55
```

Positive torque therefore required more than 50% gas. The Autoware acceleration
map normally requested much smaller values, for example 0.125. Under the old
configuration, `gas=0.125` requested about `-41.25 Nm`, so an apparently valid
positive acceleration command produced regenerative/reverse torque rather than
forward drive.

Fix:

Only in the duplicated vehicle, set:

```text
PowerTrain.Control.GasInterpret.TrqZero = 0
```

The same `gas=0.125` then produces approximately `6.875 Nm` before the final
drive, which is forward torque.

A possible steering-ratio error was also investigated. The vehicle definition
explicitly identifies `VehicleControl.Steering.Ang` as road-wheel angle, so the
1:1 road-wheel command path was retained.

### Phase 13 — Separate shutdown-only faults from runtime blockers

During Ctrl-C validation, `motion_planning_container` sometimes exited with
code `-11` in MotionVelocityPlanner/callback-group destruction.

Diagnosis:

This occurred during process teardown after SIGINT, not while the
perception-planning-control pipeline was running. It was therefore classified
as a separate shutdown teardown issue rather than the cause of the stationary
vehicle. The earlier traffic-light exit `-6`, by contrast, was a runtime launch
problem and was fixed.

The normal operating model now keeps Autoware alive between CarMaker TestRuns,
which also avoids repeatedly exercising the shutdown-only path.

## 4. Validation process and debugging of the test harness

### Package and CarMaker builds

The seven isolated packages were built with:

```bash
cd /home/klee/autoware.testvehicle
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/bin:$PATH /usr/bin/colcon build --symlink-install \
  --packages-select \
  vehiclecontrol_msgs \
  autoware_carmaker_interface \
  carmaker_fsai_vehicle_description \
  carmaker_fsai_vehicle_launch \
  carmaker_sensor_kit_description \
  carmaker_sensor_kit_launch \
  autoware_carmaker_launch
```

The duplicated CarMaker project was built with:

```bash
cd /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware
PATH=/usr/bin:/bin:$PATH ./build.sh 2
```

### Helper tests

`colcon test` initially reported zero discovered tests for the Python helper
package because of its package test-discovery metadata. This was not treated as
a pass. The test file was invoked directly with pytest and all six tests
passed:

1. command clamping and non-finite input handling;
2. map rotation and translation;
3. quaternion composition;
4. ROS stamp conversion;
5. TestRun time-regression detection;
6. sensor-before-odometry epoch detection.

### Full synthetic ROS contract

A synthetic CarMaker-format ROS publisher was connected to the real Autoware
perception, planning, control, and interface nodes. It checked:

- transient `/initialpose3d`;
- localization readiness;
- route service/state transitions;
- occupancy-grid output;
- trajectory output;
- a trajectory with positive velocity;
- actuation output;
- positive acceleration;
- autonomous availability;
- autonomous mode and control enablement.

The successful contract produced a maximum trajectory velocity of about
`4.167 m/s` and a maximum positive acceleration command of about `0.125`.

A test-harness-specific freeze was encountered during this validation.
Autoware had `use_sim_time=true`, but the synthetic harness did not publish
`/clock`. Occupancy could publish through callbacks, while time-driven state
machines did not advance. The harness was rerun with `use_sim_time=false`.
This is correct only for the synthetic test; a real CarMaker run uses
`use_sim_time=true` because CMROSIF publishes `/clock`.

A route-reset contract also exercised:

- initial sensor gating;
- restarted sensor gating;
- raw odometry time regression;
- localization reinitialization;
- safe command hold;
- a rejected route-clear request followed by retry;
- successful route clearing and command release.

## 5. Error-to-root-cause summary

| Symptom | Root cause | Resolution |
|---|---|---|
| Traffic-light container exit `-6` | Camera6/camera7 traffic-light pipelines had no supported Sprint camera input | Disabled traffic-light recognition |
| Legacy PointXYZI warning | Wrong LiDAR message/layout | Published packed PointXYZIRC `PointCloud2` |
| `/initialpose3d` missing | Ground-truth localization did not complete Autoware's initialization contract | Published first pose transient-locally |
| MRM emergency toggling | Localization/route/trajectory/control diagnostics were not ready | Completed status contracts, adjusted startup diagnostics, waited for route/trajectory |
| Stale actuation safe brake | No trajectory meant no fresh controller command | Fixed upstream route, occupancy, map, and readiness; kept watchdog |
| Empty TF source frame | Sensor entered perception before valid localization epoch | Gated sensors until odometry/TF exists |
| Map/RViz unusable after restart | `/clock` jumped backward and old route/TF data survived | Monotonic clock epochs, localization reset, route clear |
| LiDAR at 0.010 before TF at 0.012 | Sensor restarted/published before matching odometry epoch | Per-sensor epoch tracking and gating |
| Route accepted but no trajectory | Occupancy node waited for exactly equal raw and segmented timestamps | Switched to segmented-cloud laserscan occupancy |
| Pull-out failures with lateral `-2.0` | Generated lane center was offset 2 m from actual road reference | Regenerated map with offset 0 |
| Positive command but car stayed still | `TrqZero=-55` made normal low gas commands negative torque | Set duplicated vehicle's `TrqZero=0` |
| Test contract froze | Synthetic test used sim time without publishing `/clock` | Used wall time for the synthetic harness |
| `colcon test` showed zero tests | Python test discovery metadata did not register the helper tests | Ran pytest directly; six passed |
| Motion planning exit `-11` on Ctrl-C | Shutdown-time destructor/callback-group issue | Classified as teardown-only, not a driving blocker |

## 6. Final safety behavior

The integration intentionally fails safe:

- stale command timeout: `0.5 s`;
- stale-command safe brake: `0.35`;
- non-finite commands are rejected/clamped;
- sensor data is held until localization for the same epoch exists;
- a TestRun reset immediately holds commands and brakes;
- the previous route is cleared before normal operation resumes;
- autonomous mode remains unavailable without valid localization, route,
  trajectory, and controller output.

MRM and “target mode not available” messages during startup are therefore not
automatically defects. They are expected until sensor data is live, Autoware
reaches `WaitingForRoute`, the operator sets a valid goal, and a trajectory is
being published.

## 7. Final operating procedure

Start the duplicated CarMaker project:

```bash
cd /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware
./CMStart.sh 2
```

Start isolated Autoware:

```bash
cd /home/klee/autoware.testvehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autoware_carmaker_launch carmaker.launch.py
```

Load `FS_autonomous_Sprint_LTS`. Start its CMRosIF action, or manually:

```bash
cd /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware
source ros_setup.bash 2
ros2 launch hellocm hellocm.launch.py use_sim_time:=True rviz2:=False
```

Before engaging:

1. Start the CarMaker TestRun.
2. Wait for Autoware to reach `WaitingForRoute`.
3. Confirm odometry, LiDAR, localization, and occupancy are live.
4. Set a forward goal on the lane centerline.
5. Confirm `/planning/trajectory` is live and has positive velocity.
6. Confirm `/carmaker/VehicleControl` contains sensible direction, gas, brake,
   and steering values.
7. Engage autonomous mode.

After restarting a TestRun, keep Autoware and RViz running. The old route is
intentionally cleared, so wait for `WaitingForRoute` and set the goal again.

## 8. Important implementation files

Autoware side:

- `src/carmaker_integration/autoware_carmaker_interface/autoware_carmaker_interface/interface_node.py`
- `src/carmaker_integration/autoware_carmaker_interface/config/interface.param.yaml`
- `src/carmaker_integration/autoware_carmaker_launch/launch/carmaker.launch.py`
- `src/carmaker_integration/autoware_carmaker_launch/config/autoware-carmaker.yaml`
- `src/carmaker_integration/autoware_carmaker_launch/autoware_carmaker_launch/generate_sprint_map.py`
- `src/carmaker_integration/autoware_carmaker_launch/README.md`

CarMaker side:

- `ros/ros2_ws/src/hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp`
- `ros/ros2_ws/src/hellocm_cmnode/src/CMNode_ROS2_HelloCM.hpp`
- `ros/ros2_ws/src/cmrosutils/src/cmrosif.cpp`
- `ros/ros2_ws/src/cmrosutils/include/cmrosutils/cmrosif.hpp`
- `Data/Vehicle/FS-AI_IMechE_ADS-DV_2026`

## 9. Result

The final integration supplies a coherent CarMaker sensor/localization epoch
to Autoware, produces an occupancy grid, route, moving trajectory and positive
actuation command, and applies that command to the CarMaker vehicle. It also
survives repeated TestRuns without restarting Autoware and preserves a safe
brake/route-reset sequence whenever the simulation epoch changes.
