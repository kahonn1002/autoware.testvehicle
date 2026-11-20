# ISO Lane-Change Demo (`iso_lanechange_demo`)

Commissioning-only profile for the copied CarMaker TestRun
`LaneChange_ISO_Autoware`. Default profile remains `fsai_sprint`.

## 1. Vehicle / TestRun / map scope

- Demo vehicle `Autoware/DemoCar_ISO` is derived from `DemoCar`, retaining its
  manual powertrain and front/rear RGB + LiDAR mounts. Original `DemoCar` file
  is preserved; only the `Autoware/DemoCar_ISO` copy is written by the overlay.
- Copied TestRun `LaneChange_ISO_Autoware` (from `LaneChange_ISO_Camera_RSI`)
  sets `Vehicle.StartPos = 450 0` and a stationary manual-operator fallback
  (`Wait for Autoware`, 600 s, gas 0 / brake 0.5 / steer 0) so a short
  commissioning run starts before the gates. The original benchmark TestRun is
  untouched.
- Local ISO map models the three-gate layout with a 10 km/h (2.78 m/s) speed limit.
  It does not reproduce the original 90 km/h benchmark maneuver.
- ISO PCD is synthetic flat-road/gate geometry only, for map loading and RViz
  context. It is not an NDT scan and not localization-grade.

## 2. Build

Autoware side (seven packages: `vehiclecontrol_msgs`,
`autoware_carmaker_interface`, `autoware_carmaker_launch` plus four demo
packages `carmaker_demo_vehicle_description`, `carmaker_demo_vehicle_launch`,
`carmaker_demo_sensor_kit_description`, `carmaker_demo_sensor_kit_launch`):

```bash
cd /home/klee/autoware.testvehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
PATH=/usr/bin:/bin:$PATH /usr/bin/colcon build --symlink-install --base-paths src/carmaker_integration --packages-select vehiclecontrol_msgs autoware_carmaker_interface autoware_carmaker_launch carmaker_demo_vehicle_description carmaker_demo_vehicle_launch carmaker_demo_sensor_kit_description carmaker_demo_sensor_kit_launch
```

The integration build does not rebuild existing Autoware planners. If their
`CMAKE_BUILD_TYPE` was empty (no compiler optimization), rebuild the measured
planning hot path before timing a real-RSI run:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 PATH=/usr/bin:/bin:$PATH /usr/bin/colcon build --symlink-install --executor sequential --packages-select autoware_lanelet2_extension autoware_route_handler autoware_behavior_path_planner_common autoware_behavior_path_planner autoware_behavior_path_static_obstacle_avoidance_module autoware_behavior_path_goal_planner_module autoware_path_optimizer --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

This changes build optimization, not planning algorithms or the 1-second
trajectory freshness limit. The low compile concurrency limits memory use.

CarMaker overlay (dry-run first, then normal). Both commands target only the
duplicate project; installer writes backups beside the project:

```bash
/usr/bin/python3 src/carmaker_integration/carmaker_overlay/install_overlay.py --project /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware --dry-run
/usr/bin/python3 src/carmaker_integration/carmaker_overlay/install_overlay.py --project /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware
```

Changed CM-side packages: `vehiclecontrol_msgs`, `hellocm_cmnode`,
`carmaker_rsds_client`. Build these in the duplicated project:

```bash
cd /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware/ros/ros2_ws
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/bin:$PATH /usr/bin/colcon build --packages-select vehiclecontrol_msgs hellocm_cmnode carmaker_rsds_client --cmake-args -DCARMAKER_VER:STRING=15.0 -DCARMAKER_DIR:PATH=/opt/ipg/carmaker/linux64-15.0
```

Source `ros_setup.bash 2` from the CarMaker project root before launch.
`vehiclecontrol_msgs` must be rebuilt in BOTH the Autoware and CarMaker
workspaces after any `.msg` change.

## 3. Launch

```bash
cd /home/klee/autoware.testvehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autoware_carmaker_launch carmaker.launch.py profile:=iso_lanechange_demo
```

`camera_bridge:=false` disables only the Autoware camera relays; it does not
disable CarMaker RSI cameras.
Normal launch defaults to `publish_vehicle_control:=true`. The stationary
diagnostic below explicitly disables it; that diagnostic cannot drive the car.

In a second terminal, start the CarMaker Office and load `LaneChange_ISO_Autoware`:

```bash
cd /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware
./CMStart.sh 2
```

Use its CMRosIF launch action, or run `source ros_setup.bash 2` followed by
`ros2 launch hellocm hellocm.launch.py use_sim_time:=True rviz2:=False`.

## 4. Provisional parameters (calibration required)

- Sensor TF has a single owner: the robot-state publisher;
  `Autoware.SensorTFExternal = 1` in the copied vehicle disables duplicate
  CarMaker/RSDS transforms. Raw camera frames carry a unique TestRun epoch tag;
  raw odometry carries the same ID (`map__epoch_<id>`). The Humble-compatible
  relay requires an exact match, removes both tags from Autoware outputs, and
  rejects old frames even if that camera was never received before reset.
- IMU here is virtual ground-truth, not the physical YRS sensor yet.
- Tire/steering ratio 13.66 with sign -1 is provisional.
- Accel/brake maps are placeholders requiring step-test calibration. There is
  no 90 km/h validation.

## 5. Current evidence

- 114 pytest tests passed; seven integration plus three CM packages built.
  Seven planning hot-path packages also rebuilt successfully in Release mode.
- DDS `smoke_interface --data-only` passes with no CarMaker commands:
  91 poses/images/IMUs/accels, 0 VehicleControl messages. ISO profile
  explicitly outputs `/sensing/imu/imu_data`; bridge uses reliable output
  QoS for AEB (raw inputs remain best-effort).
- The separate `LaneChange_ISO_Autoware_NoRSI` commissioning copy passed a
  12-second vehicle-control check, reached 0.701 m/s, and stopped. This verifies
  the manual clutch/gear/pedal path; it is not an autonomous lane-change run.
  Travel was about 0.2 m: this is only a basic actuator-response check.
- A separate 12-second NoRSI watchdog check stopped sending commands at 9 s.
  It reached 0.693 m/s and stopped; CarMaker logged the fallback explicitly:
  `gas=0 brake=0.5 gear=0 clutch=1`. Report:
  `/tmp/iso-watchdog-control-report.json`, log:
  `/tmp/iso-carmaker-smoke-d03dcp_g/carmaker.log`.
  The 201 cloud messages in this run were empty; real LiDAR was disabled.
- xacro and ISO map load; timed ~45 s Autoware startup with no sensors cannot
  engage autonomous mode (expected).
- Shutdown SIGSEGV remains unresolved: previously observed in the motion-planning
  container; the final stationary retest observed it in `mission_planner_container`
  after SIGINT. Overall harness status fails on this crash even when runtime
  readiness passes. No common root cause or impact on driving is established.
- An 8 s stationary attempt of the copied RSI TestRun via standalone CLI
  aborted before any sensor message with
  `LidarRSI 0: Not enough free GPUSensors available`. GUI Office supplies
  the GPU-sensor startup absent from standalone CLI, so this earlier abort
  does not confirm a license fault.
- Real front/rear RSI camera streams ran through Office; their Autoware image
  outputs were not explicitly measured in that timing probe. No completed
  autonomous lane change is claimed.

### 5.1 Stationary no-actuation troubleshooting (Office, real RSI, 60 s)

- First full real-RSI Office stationary run: timing
  `/tmp/iso-timing-office-camera-on.json`, logs
  `/tmp/iso-stationary-re99jmxh`. Route accepted; zero commands, no Auto,
  peak speed 1.07e-6 m/s.
- Raw LiDAR delivered 1001 clouds; sampled clouds contained finite nonzero XYZ.
  The validator's map-filtered cloud was empty, but object messages continued
  downstream. That warning alone does not prove a broken planning input.
- Downstream stall: trajectory ~2.13 s p95, BPP processing 1701 ms p95;
  final trajectory only 2 messages, no autonomous availability — not passed.
- All selected planning libraries were previously compiled without `-O` flags.
  After the targeted Release rebuild, the final 60 s real-RSI retest passed
  **runtime readiness**: BPP p95 processing 122.8 ms, 284 observed final
  trajectories with steady p95 age 0.745 s (maximum overall age 0.799 s),
  and planned speed up to 2.78 m/s. Autonomous availability was observed and
  1120 trajectory-follower control messages were received.
- The final probe observed 654 raw LiDAR clouds with valid XYZ samples and
  1239 standardized IMU messages. These are sampled observations, not wire rates.
  It sent zero VehicleControl messages, never entered autonomous mode, and
  peak measured speed remained 1.07e-6 m/s. No autonomous driving was tested.
- Final report: `/tmp/iso-timing-after-release.json`, logs
  `/tmp/iso-stationary-3y6g88tj`. `readiness_passed=true`, but
  `cleanup_passed=false` / overall `passed=false` because of the shutdown
  SIGSEGV. All three owned GPU processes exited; ports 2210/2211 were free.
- The probe keeps only the newest samples and a limited set of pipeline
  subscriptions. An earlier broad debug-topic probe overloaded after the
  optimized rebuild; its end-to-end latency figures were not accepted.
- The harness now tracks its Office GPU children with verified process
  identity and stable Linux pidfds, and stops only those children. Earlier
  leaked test renderers occupied camera ports and caused a separate startup
  abort; the six verified test processes were stopped, without touching other
  applications. Do not resolve occupied ports by killing all MovieNX processes.
- Safe probe from the CM duplicate root. It verifies
  `publish_vehicle_control=false` before starting, requests only a route,
  never engages (do not manually engage during test), refuses existing
  CarMaker/MovieNX processes or a non-empty domain, and validates neutral /
  clutch-open / brake fallback:

```bash
cd /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware
source /opt/ros/humble/setup.bash
source ros_setup.bash 2
source /home/klee/autoware.testvehicle/install/setup.bash
PATH=/usr/bin:/bin:$PATH /usr/bin/python3 /home/klee/autoware.testvehicle/src/carmaker_integration/carmaker_overlay/stationary_readiness.py --project /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware --seconds 60 --domain-id 57 --camera-bridge true --office --report /tmp/iso-stationary-report.json
```

- `smoke_interface --data-only` likewise sends no CarMaker commands. No
  autonomous drive or 90 km/h claim.

## 6. Bounded smoke harness

```bash
cd /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware
source ros_setup.bash 2
source /home/klee/autoware.testvehicle/install/setup.bash
ROS_DOMAIN_ID=54 /usr/bin/python3 /home/klee/autoware.testvehicle/src/carmaker_integration/carmaker_overlay/smoke_runtime.py --project /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware --seconds 8 --report /tmp/iso-runtime-report.json
```

Defaults: `--report /tmp/...`, `--seconds 10` (range 1-30). Actual RSI run log:
`/tmp/iso-carmaker-smoke-gtog3t8v`. The harness stops only the process groups
it started; it issues no broad kill/delete commands.

For a separate commissioning check with GPU sensors disabled, add
`--no-rsi --drive-check --seconds 12 --report /tmp/iso-vehicle-control-report.json`.
The installer creates explicit `_NoRSI` copies; the main profile retains all RSI
sensors. This check commands low throttle and ends with braking.

To test process-loss protection, also add `--watchdog-check` (minimum 8 s).
That mode sends no control messages for the final 3 s, and passes only if the
simulator watchdog diagnostic is observed and the vehicle is stopped.

Unit checks and the DDS smoke test:

```bash
cd /home/klee/autoware.testvehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q src/carmaker_integration/autoware_carmaker_interface/test src/carmaker_integration/autoware_carmaker_launch/test
ROS_DOMAIN_ID=55 /usr/bin/python3 src/carmaker_integration/autoware_carmaker_interface/test/smoke_interface.py
```

## 7. RViz engage attempt (when sensors are healthy)

- In RViz set a 2D Goal Pose in the map frame ahead on the lane centerline
  (local ISO map, e.g. x ~650, y 0).
- Readiness = route received plus live `/planning/trajectory`; check
  `/api/routing/state`, `/planning/mission_planning/route`, trajectory hz.
- Start at low speed first. Exact ISO steering-only handoff needs a separate
  procedure and is still not implemented.

## 8. Explicitly unverified

- Closed-loop lane change, 90 km/h behavior, real RSI camera image path, NDT
  localization on synthetic PCD, actuation maps, steering sign/ratio,
  GPU/LidarRSI failure root cause, shutdown segfault fix, multi-run clock
  behavior for the ISO TestRun.

## 9. Process record

[Visual build/debug journal](../../docs/process-journal/iso-lanechange/index.html)
and its [canonical evidence data](../../docs/process-journal/iso-lanechange-source.json)
record this session's construction steps, corrective changes and remaining blockers.
