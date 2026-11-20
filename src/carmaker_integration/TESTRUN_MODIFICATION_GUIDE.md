# Modifying a CarMaker TestRun for Autoware

How to take a CarMaker 15 TestRun (your own, or one from the CarMaker
`Examples`) and change its vehicle, sensors, environment, road, traffic or
manoeuvre so that Autoware can drive it. Each section lists what to edit in
CarMaker and what has to change on the Autoware side to match.

Paths used throughout:

| Name | Path |
|---|---|
| CarMaker install | `/opt/ipg/carmaker/linux64-15.0` |
| CarMaker project | `/home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware` (`$PRJ` below) |
| CarMaker ROS workspace (CMNode) | `$PRJ/ros/ros2_ws` |
| Autoware workspace | `/home/klee/autoware.testvehicle` (`$AW` below) |
| Integration packages | `$AW/src/carmaker_integration` |

```bash
export PRJ=/home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware
export AW=/home/klee/autoware.testvehicle
```

---

## 0. Overview

A TestRun change can touch up to four places:

1. **CarMaker project files** (`$PRJ/Data/...`): the TestRun, vehicle and road
   files. These are CarMaker "infofiles": plain `Key = Value` text that CarMaker
   Office edits for you.
2. **CarMaker ROS node (CMNode)** (`$PRJ/ros/ros2_ws`). It publishes odometry,
   IMU, lidar and steering, starts one camera client per camera, and receives
   `VehicleControl` from Autoware. It reads the vehicle file at simulation
   start, so most sensor changes need no code change here.
3. **The Autoware integration** (`$AW/src/carmaker_integration`): vehicle
   description, sensor kit, interface parameters and launch profile.
4. **The converter** (`testrun_to_autoware`). It turns the TestRun into an
   Autoware map, scenario and profile. Run it again whenever the road, route,
   start position, cones/pylons or vehicle change.

What each kind of change needs:

| You change… | CarMaker side | Autoware side | Re-run converter? | Rebuild? |
|---|---|---|---|---|
| Road, route, start position, cones, pylons | TestRun (Scenario Editor) | nothing by hand | **yes** (`--force`) | `--build` |
| Vehicle to another mapped vehicle | TestRun `Vehicle =` | nothing | **yes** | `--build` |
| Vehicle to a new vehicle | vehicle file copy | vehicle description, interface params, profile | **yes** | full integration build |
| Add, move or remove a camera or lidar | vehicle file (`Sensor.*`, `SensorCluster.*`) | sensor kit calibration, interface camera list | no | integration build |
| Camera image settings (resolution, FoV, exposure) | vehicle file `Sensor.Param.*` | nothing (camera info is derived) | no | no |
| Environment (time, sun, fog, rain, wind) | TestRun `Env.*` | nothing | no | no |
| Moving traffic or obstacles | TestRun `Traffic.*` | perception, if Autoware should react | yes (they are listed in `scenario.yaml`) | `--build` |
| Manoeuvre or driver | TestRun `DrivMan.*` / `Driver.*` | nothing | no | no |

---

## 1. Ground rules

- **Use CarMaker 15 only.** The converter reads roads with CM15's IPGRoad API.
  A TestRun saved by CarMaker 13 or 14 can fail with an error such as
  `Adding junction arm - failed, 'lengthOnLink' not equal to zero`. Open it
  once in CarMaker 15 Office and save it, then convert it.
- **Edit copies, never originals.**
  - **TestRuns:** copy into `$PRJ/Data/TestRun/` with an `_Autoware` suffix,
    for example `Slalom18m_Autoware`.
  - **Vehicles:** copy into `$PRJ/Data/Vehicle/Autoware/`, for example
    `Autoware/DemoCar_ISO`. The `Examples/...` and `ExamplesMM/...` folders
    in the CarMaker install are read-only references.
- **Cars only.** `ExamplesMM` (MotorcycleMaker) and `ExamplesTM` (TruckMaker)
  TestRuns use vehicles Autoware cannot drive. Keep their road, but put a car
  in them (section 3).
- **Run RSI sensors from CarMaker Office.** The RSI sensors are GPU-rendered
  (`CameraRSI`, `LidarRSI`). Start CarMaker Office (`./CMStart.sh 2`) and
  start the run from there. A headless standalone CLI start does not set up
  the GPU sensors. One such attempt aborted with
  `LidarRSI 0: Not enough free GPUSensors available`. The same vehicle with
  two cameras and a lidar runs fine from Office, for example in
  `open_loop_test` and `LaneChange_ISO_Autoware`.
- **Don't hand-edit generated profiles.** Files with
  `generated_by: testrun_to_autoware` at the top are overwritten. Change the
  TestRun or the base profile, then run
  `testrun_to_autoware <TestRun> --force --build`.
- **Close CarMaker Office before editing a file it has open.** Or reload the
  file afterwards, so Office does not overwrite your edit when it saves.

---

## 2. Bring the TestRun into the project

### 2.1 In CarMaker Office (recommended)

1. Start Office from the project: `cd $PRJ && ./CMStart.sh 2`.
2. Choose **File → Open**, and in the file browser switch the source from
   the project to the CarMaker installation (or browse to the other
   project). Pick the TestRun, for
   example `Examples/VehicleDynamics/Handling/Slalom18m`.
3. Choose **File → Save As**, select the project's `Data/TestRun`, and save it
   as `<Name>_Autoware`.

### 2.2 From the shell

```bash
cp /opt/ipg/carmaker/linux64-15.0/Data/TestRun/Examples/VehicleDynamics/Handling/Slalom18m \
   $PRJ/Data/TestRun/Slalom18m_Autoware
```

If the TestRun uses an external road file (`Road.FName = something.rd5`
instead of embedded `Road.*` keys), copy that file into `$PRJ/Data/Road/` too.

### 2.3 What the converter needs from the TestRun

| Requirement | Key(s) | Notes |
|---|---|---|
| A route for the ego vehicle | `Vehicle.Routing.Type = Route`, `Vehicle.Routing.ObjId` | Choose a different one with `--route <name or id>`. |
| Route-type start position | `Vehicle.StartPos.Type = Route`, `Vehicle.StartPos = <s> <t>` | Other start types are rejected. `s` is the distance along the route; `t` is the lateral offset from the lane centre. |
| A vehicle that maps to an Autoware profile | `Vehicle = ...` | See section 3.1, or pass `--base-profile`. |
| A road IPGRoad 15 can read | embedded `Road.*` or `Road.FName` | |

---

## 3. Change the vehicle

### 3.1 Use a vehicle that already has an Autoware profile (easiest)

| CarMaker `Vehicle =` | Autoware base profile | Autoware vehicle and sensor packages |
|---|---|---|
| `FS-AI_IMechE_ADS-DV_2026` | `fsai_sprint` | `carmaker_fsai_vehicle_*`, `carmaker_sensor_kit_*` |
| `Examples/DemoCar` | `iso_lanechange_demo` | `carmaker_demo_vehicle_*`, `carmaker_demo_sensor_kit_*` |
| `Autoware/DemoCar_ISO` | `iso_lanechange_demo` | same as above |
| `Autoware/DemoCar_ISO_NoRSI` | `iso_lanechange_demo` | same, without RSI sensors |

For a DemoCar TestRun, prefer `Autoware/DemoCar_ISO` over `Examples/DemoCar`.
It is the same car, but it sets `Autoware.SensorTFExternal = 1`, which lets
Autoware own the sensor transforms (section 4.4). This mapping lives in
`VEHICLE_PROFILES` in
`autoware_carmaker_launch/autoware_carmaker_launch/testrun/profiles.py`.

To swap the vehicle in the GUI: open the TestRun, click the **Vehicle** field
on the main panel, choose the vehicle and save. In the file, it is one line:

```
Vehicle = Autoware/DemoCar_ISO
```

The vehicle needs matching per-vehicle TestRun keys. A car needs four tyre
entries and may carry a load; a motorcycle has two tyres:

```
Trailer =
Tire.0 =
Tire.1 =
Tire.2 =
Tire.3 =
VehicleLoad.0.mass = 70
VehicleLoad.0.pos = 2.0 0.35 0.75
```

An empty `Tire.N =` means "use the vehicle's own tyre". The simplest
reliable approach is to copy the whole non-road part of a working Autoware
TestRun of the same vehicle (for example `LaneChange_ISO_Autoware`) and keep
only the `Road.*`, `Route.*`, `Vehicle.StartPos*` and `Vehicle.Routing*`
keys from the new TestRun. That is how `Slalom18m_Autoware` was made
(section 11).

### 3.2 Use a new CarMaker vehicle (needs Autoware work)

1. **CarMaker:** copy the vehicle into `$PRJ/Data/Vehicle/Autoware/<Name>`
   and add:
   ```
   Autoware.SensorTFExternal = 1
   ```
2. **Autoware vehicle description:** copy
   `carmaker_demo_vehicle_description` or `carmaker_fsai_vehicle_description`
   to a new package. Fill `config/vehicle_info.param.yaml` from the CarMaker
   vehicle file. All CarMaker coordinates are in the Fr1 frame: x forward
   from the vehicle's rear end, y left, z up.

   | Autoware `vehicle_info` | From CarMaker vehicle file |
   |---|---|
   | `wheel_base` | `Wheel.fl.pos[x] − Wheel.rl.pos[x]` |
   | `wheel_tread` | `Wheel.fl.pos[y] − Wheel.fr.pos[y]` |
   | `front_overhang` | `max(OuterSkin x) − Wheel.fl.pos[x]` |
   | `rear_overhang` | `Wheel.rl.pos[x] − min(OuterSkin x)` |
   | `vehicle_width` | `abs(OuterSkin y1 − OuterSkin y2)` |
   | `vehicle_height` | `max(OuterSkin z)` |
   | `wheel_radius`, `wheel_width` | from the tyre size, for example `205_55R16` |
   | `max_steer_angle` | the steering system's maximum tyre angle |

   `Vehicle.OuterSkin = x1 y1 z1 x2 y2 z2` gives two opposite corners of the
   body box. The converter computes the same quantities and warns when
   `vehicle_info` differs by more than 5 cm. Read the warnings it prints.
3. **Interface parameters:** copy
   `autoware_carmaker_interface/config/demo_iso.interface.param.yaml` and
   set:

   | Parameter | Meaning |
   |---|---|
   | `source_base_offset_x` | CarMaker odometry reports `Car.Fr1.t_0`, the Fr1 origin at the vehicle's rear end. Autoware's `base_link` is the rear axle, so set this to **`Wheel.rl.pos[x]`** (DemoCar: `0.863`). |
   | `steering_command_ratio`, `steering_command_sign`, `steering_feedback_*` | Handwheel to tyre angle (DemoCar: 13.66 and −1). |
   | `max_steering_angle`, `max_steering_rate` | Limits sent to CarMaker. |
   | `use_manual_transmission` | `true` for a manual gearbox, such as the DemoCar. |
   | `enable_cameras`, `require_camera_epoch` | See section 4. |
   | `hardware_id` | Free text shown in diagnostics. |
4. **Throttle and brake maps:** copy the vehicle launch package's
   `raw_vehicle_cmd_converter.param.yaml` and its CSV maps. They are not
   calibrated yet for either existing car, which is why the converter caps
   speed at 20 km/h.
5. **Base profile:** create
   `autoware_carmaker_launch/config/profiles/<name>.yaml`:
   ```yaml
   vehicle_model: <your_vehicle_description_package_without__description>
   sensor_model: <your_sensor_kit_package_without__description>
   vehicle_launch_package: <your_vehicle_launch_package>
   interface_package: autoware_carmaker_interface
   interface_params: <your>.interface.param.yaml
   converter_params: raw_vehicle_cmd_converter.param.yaml
   map_path: maps/<some_map>
   camera_bridge: true
   ```
6. **Map the vehicle:** either add `"Autoware/<Name>": "<name>"` to
   `VEHICLE_PROFILES` in `testrun/profiles.py`, or pass
   `--base-profile <name>` every time you convert.

> **Check the FS-AI car.** `FS-AI_IMechE_ADS-DV_2026` has its rear axle at
> `Wheel.rl.pos[x] = 0.5637`, but its interface file (`interface.param.yaml`)
> leaves `source_base_offset_x` at 0. Autoware's `base_link` may therefore
> sit about 0.56 m behind the real rear axle. Its `vehicle_info` also
> differs from CarMaker:
>
> | Parameter | Autoware | CarMaker |
> |---|---|---|
> | `wheel_base` | 1.600 | 1.53 |
> | `front_overhang` | 0.567 | 0.806 |
> | `rear_overhang` | 0.483 | 0.564 |
> | `vehicle_height` | 1.0 | 0.78 |
>
> Confirm and fix both before tuning the FS-AI controller.

---

## 4. Change the sensors

Sensors live in the **vehicle file**, not the TestRun. Edit them in Office's
vehicle data set (the *Sensors* tab for mounts, and the sensor parameter
dialog for types and settings), or in the file directly. Three groups of keys work
together:

```
Sensor.Param.<p>.*     what the sensor is (type, resolution, FoV, …)   count: Sensor.Param.N
Sensor.<i>.*           where it is mounted and which Param/Cluster     count: Sensor.N
SensorCluster.<c>.*    GPU render group: type, TCP port, cycle time    count: SensorCluster.N
```

Real example from `Autoware/DemoCar_ISO`, the front camera:

```
Sensor.Param.3.Type = CameraRSI
Sensor.Param.3.Name = CARS00
Sensor.Param.3.Resolution = 1920 1080
Sensor.Param.3.FoV = 90
Sensor.Param.3.OutputFormat = rgb
...
Sensor.3.name = VehSensor_0          <- becomes the ROS topic /VehSensor_0/image_raw
Sensor.3.rot = 0.0 0.0 0.0           <- roll pitch yaw [deg], Fr1 axes
Sensor.3.pos = 2.2 0.0 1.5           <- x y z [m] in Fr1 (from the vehicle rear end)
Sensor.3.Mounting = Fr1A
Sensor.3.Active = 1
Sensor.3.Ref.Param = 3               <- uses Sensor.Param.3
Sensor.3.Ref.Cluster = 2             <- rendered by SensorCluster.2
...
SensorCluster.2.Type = CameraRSI
SensorCluster.2.ClientType = UserDefined
SensorCluster.2.Socket = 2211        <- TCP port the camera client connects to
SensorCluster.2.CycleTime = 60       <- ms per image (about 16.7 Hz)
SensorCluster.2.UseMovieNX = 1
```

### 4.1 How CarMaker sensors reach Autoware

| Sensor | CMNode publishes | Autoware interface relays to |
|---|---|---|
| Vehicle pose (`Car.Fr1`) | `/carmaker/odometry` | `/localization/kinematic_state`, pose, TF |
| Inertial / IMU | `/carmaker/imu` | `/sensing/imu/...` (per `output_imu_topic`) |
| First `LidarRSI` only | `/carmaker/lidar/front/points` | `/sensing/lidar/concatenated/pointcloud` |
| Steering | `/carmaker/steering_angle` | `/vehicle/status/steering_status` |
| Each **active** `CameraRSI` | `/<Sensor.i.name>/image_raw`, `/camera_info` | `/sensing/camera/<name>/image_raw`, `/camera_info` |

For cameras, CMNode starts one `carmaker_rsds_client` per active camera. It
passes the mount pose, FoV and resolution, and derives the camera
intrinsics from them, so you never write `camera_info` by hand.

### 4.2 Add a camera (step by step)

Example: add a third, left-looking camera `VehSensor_2` to the copied
`Autoware/DemoCar_ISO`.

**CarMaker vehicle file**

1. Add a camera parameter set: copy the whole `Sensor.Param.4.*` block to
   `Sensor.Param.5.*`, then change `Name = CARS02` and any image settings.
   Increase `Sensor.Param.N` to `6`.
2. Add a mount: copy the `Sensor.3.*` block to `Sensor.5.*` and set:
   ```
   Sensor.5.name = VehSensor_2
   Sensor.5.pos = 2.0 0.8 1.3
   Sensor.5.rot = 0.0 0.0 90
   Sensor.5.Active = 1
   Sensor.5.Ref.Param = 5
   Sensor.5.Ref.Cluster = 3
   ```
   Increase `Sensor.N` to `6`.
3. Add a **new cluster for it**: copy the `SensorCluster.2.*` block to
   `SensorCluster.3.*` with a new port, `SensorCluster.3.Socket = 2212`.
   Increase `SensorCluster.N` to `4`.
   - **One camera per cluster.** CarMaker's camera-to-ROS client sends only
     one camera per cluster. With two or more in a cluster, only one reaches
     ROS, and maybe not the one you intended (see
     `$PRJ/ros/ros2_ws/src/carmaker_rsds_client/README.md`).
   - **Unique port per camera cluster** (2210, 2211, 2212 …).
4. Check GPU load. Each camera is rendered on the GPU every `CycleTime`, and
   1920×1080 images are expensive. Lower `Resolution` or raise `CycleTime`
   if the frame rate drops.

**Autoware sensor kit** (`carmaker_demo_sensor_kit_description`)

5. In `config/sensor_kit_calibration.yaml`, add a mount under
   `carmaker_vehicle_origin`, which is CarMaker's Fr1 frame placed
   `-0.863` m behind `base_link`. Copy `Sensor.5.pos` directly, and
   `Sensor.5.rot` converted from degrees to radians:
   ```yaml
   carmaker_vehicle_origin:
     camera_left_mount:
       x: 2.0
       y: 0.8
       z: 1.3
       roll: 0.0
       pitch: 0.0
       yaw: 1.5707963267948966
   camera_left_mount:
     VehSensor_2:          # optical frame: z forward, x right, y down
       x: 0.0
       y: 0.0
       z: 0.0
       roll: -1.5707963267948966
       pitch: 0.0
       yaw: -1.5707963267948966
   ```
   CarMaker and ROS share the axis convention (x forward, y left, z up), so a
   positive pitch tilts the camera down in both.
6. In `urdf/sensors.xacro`, add the two joints and links
   `camera_left_mount` and `VehSensor_2`. Copy the `camera_front_mount` lines
   and rename them.

**Autoware interface**

7. The camera list is currently hard-coded in
   `autoware_carmaker_interface/autoware_carmaker_interface/interface_node.py`
   (around line 220):
   ```python
   for name, source, frame in (
       ("front", "VehSensor_0", "VehSensor_0"),
       ("rear", "VehSensor_1", "VehSensor_1"),
       ("left", "VehSensor_2", "VehSensor_2"),   # new
   ):
   ```
   `name` is the Autoware topic name (`/sensing/camera/left/...`), `source`
   is the CarMaker `Sensor.<i>.name`, and `frame` is the TF frame from step 5.
   *(Planned: move this list into the interface parameter file so this step
   becomes configuration.)*
8. Keep `enable_cameras: true` and `require_camera_epoch: true` in the
   interface parameters, and `camera_bridge: true` in the profile.

**Build and check**

9. Build:
   ```bash
   cd $AW && source /opt/ros/humble/setup.bash && source install/setup.bash
   PATH=/usr/bin:/bin:$PATH /usr/bin/colcon build --symlink-install \
     --base-paths src/carmaker_integration \
     --packages-select autoware_carmaker_interface carmaker_demo_sensor_kit_description \
                       carmaker_demo_sensor_kit_launch autoware_carmaker_launch
   ```
10. Run (section 9), then check:
    ```bash
    ros2 topic list | grep -E 'VehSensor_2|camera/left'
    ros2 topic hz /sensing/camera/left/image_raw
    ros2 run tf2_ros tf2_echo base_link VehSensor_2
    ```
    In RViz, add an *Image* display on `/sensing/camera/left/image_raw`.
    The repo's camera readiness check also works:
    `carmaker_overlay/stationary_readiness.py --camera-bridge true --office ...`
    (see `ISO_LANECHANGE.md` §5).

### 4.3 Move, change or remove a sensor

- **Move a camera or lidar:** change `Sensor.<i>.pos/rot` in CarMaker **and**
  the matching mount in `sensor_kit_calibration.yaml`. They must agree, or
  Autoware will project the data in the wrong place.
- **Image settings** (`Resolution`, `FoV`, `Exposure.*`, `OutputFormat`,
  `SensorNoise`, `WeatherLens*`): CarMaker only. `camera_info` follows
  automatically.
- **Disable a camera without deleting it:** `Sensor.<i>.Active = 0`. CMNode
  then starts no client for it. Remove it from the interface list too, or
  the relay will wait for frames that never come. That doesn't block driving,
  but readiness checks will report it.
- **Lidar:** only the **first** `LidarRSI` is published, on
  `/carmaker/lidar/front/points`. The beam pattern is
  `Sensor.Param.<p>.Beams.FName`, and the scan period is the lidar cluster's
  `CycleTime`. Keep `lidar_frame` (interface) and `carmaker_lidar` (sensor
  kit) consistent with `Sensor.<i>.pos`.
- **Collision sensor, extra IMUs:** these are not used by Autoware.

### 4.4 Sensor transforms (`Autoware.SensorTFExternal`)

- **`= 1`** (the `Autoware/*` vehicles):
  - Autoware's robot-state publisher owns all sensor transforms, built from
    the sensor kit.
  - CMNode publishes none, and appends `__epoch_<id>` to camera frame ids so
    the interface can drop images from a previous TestRun start.
  - The interface strips that tag before republishing.
  - This needs `require_camera_epoch: true` in the interface parameters.
- **`= 0` or missing** (`Examples/DemoCar`, `FS-AI_IMechE_ADS-DV_2026`): each
  camera client publishes its own `Fr1A → <camera>` transform. Combined with
  Autoware's sensor kit, that gives the camera frame two parents. Set it to
  `1` in your vehicle copy.

### 4.5 Cameras on the FS-AI car

The FS-AI car already has `front_camera_rgb` and `front_camera_depth`
(3480×1080 and depth, 15° down). Its profile has `camera_bridge: false`.
To use them in Autoware:
1. In a copy of the vehicle, set `Autoware.SensorTFExternal = 1`.
2. Add these camera names to the interface camera list (step 7 above).
3. Add camera mounts to `carmaker_sensor_kit_description`, which currently
   only has `carmaker_lidar` and `carmaker_imu`.
4. Create an interface parameter file with `enable_cameras: true`.

---

## 5. Change the environment

The environment is in the **TestRun** (`Env.*`). In Office it is under
**Parameters → Environment**. It changes what the cameras and lidar see,
not the road, so **no re-conversion or Autoware change is needed**.

| What | Keys (current values in `LaneChange_ISO_Autoware`) |
|---|---|
| Date and time (sun position, lighting) | `Env.StartTime.Year/Month/Day/Hour/Min/Sec` (2014-01-01 12:00), `Env.StartTime.DeltaUTC` |
| Sun | `Env.Sun.Position = angleDefinition`, `Env.Sun.Azimuth = 45`, `Env.Sun.Elevation = 45` |
| Clouds | `Env.Cloud.Kind = simple`, `.Height`, `.Intensity = 0.1`, `.Velocity`, `.Angle` |
| Fog | `Env.FogActive = 0`, `Env.VisRangeInFog = 10000` (m), `Env.FogUseSkyColor`, `Env.FogColor` |
| Rain or snow | `Env.Precipitation.Type = None`, plus intensity keys once you choose a type |
| Wind | `Env.Wind.Kind = none`, `Env.Wind.Velocity` (m/s), `Env.Wind.Angle` (deg) |
| Air | `Env.Temperature`, `Env.AirDensity`, `Env.AirPressure`, `Env.AirHumidity` |
| Solar radiation | `Env.SolarRadiation` |

Tips:
- Set precipitation and fog **in the GUI** first and save. The dialog writes
  the complete, valid key set for the chosen type. Then copy those keys to
  other TestRuns.
- Night runs (`Env.StartTime.Hour = 22`) need the camera exposure changed too
  (`Sensor.Param.<p>.Exposure.Mode`, `ISO`, `ShutterSpeed`). Otherwise images
  come out black.
- Fog and rain affect `CameraRSI` and `LidarRSI` only when the lens and
  weather options are on (`Sensor.Param.<p>.WeatherLensEnabled = 1`).
- **Wind** is the one environment setting that affects the vehicle dynamics
  directly, through crosswind on the car body. Expect more lateral error in
  Autoware's path tracking.
- Autoware runs here with perception **off**, so weather changes the images
  but not how Autoware drives. It localizes from CarMaker's ground-truth pose
  and keeps to the map lanes.

---

## 6. Change the road, route or start position

Edit in Office with the **Scenario Editor** (the road button on the main
panel), then save the TestRun. Afterwards **always re-convert** with
`--force`.

| Change | Where | Converter behaviour |
|---|---|---|
| Road geometry, lanes, junctions | Scenario Editor; embedded `Road.*` or `Road.FName = <x>.rd5` | All drivable lanes become lanelets. Junctions become connector lanes with `turn_direction`. |
| Pylon gates (ISO lane change, slalom) | Scenario Editor, driver pylon markers (`Road.RL.<n>.Marker.<k>.Type = DrvPylon`, `Param = s 0 t 100 1 width`) | Corridor mode, `boundary=pylons`. The lanelet boundaries follow the gates. |
| Cones (Formula Student) | Traffic objects with templates such as `TrafficCone_*_Blue` / `_Yellow` / `_Orange` | Corridor mode, `boundary=cones`. Blue = left, yellow = right, orange = markers. `--cone-margin` insets the boundary from the cone centres. |
| Speed limits | Scenario Editor → speed markers (`DrvSpeed`) | Copied into the lanelets and capped by `--max-speed-kmh` (default 20). |
| Ego route | `Vehicle.Routing.ObjId`, or `--route` | Network mode adds waypoints so Autoware follows *this* route, not the shortest one. |
| Start position | `Vehicle.StartPos = <s> <t>` (Route type) | Becomes the Autoware initial pose, with `source_base_offset_x` added. |
| Goal | the route end (`--goal-margin`, default 10 m before it) | On closed loops, the goal is placed behind the start. |
| Named parameters | `$Name=default` values in the TestRun | Override with `--param Name=value`. |

What is **not** converted: traffic lights, stop lines and right of way.
Autoware won't stop for CarMaker signals. Figure-eight courses (SkidPad)
convert but warn that the laps overlap.

---

## 7. Change traffic and objects

Traffic is in the **TestRun** (`Traffic.N`, then `Traffic.<k>.*`): name,
template (`Traffic.<k>.Template.FName`), route and start
(`Traffic.<k>.StartPos = <s> <t>`), and manoeuvres
(`Traffic.<k>.Man.*`). In Office it is **Parameters → Traffic**.

- **Cones and pylons** (templates containing "cone" or "pylon") are used as
  corridor boundaries (section 6) and added to the point cloud map.
- **Static obstacles and moving vehicles** are listed in `scenario.yaml`
  under `traffic_objects` but are **not** put into the map. Autoware runs
  with perception off and therefore won't avoid them. Making Autoware react
  to them needs a perception pipeline (a separate project).
- After adding or moving cones, re-convert with `--force`.

---

## 8. The manoeuvre: hand control to Autoware

CarMaker's own driver (IPGDriver) must not drive, or it fights Autoware's
commands. Replace the TestRun's `DrivMan.*` block with the "stationary
fallback" used by every `*_Autoware` TestRun:

```
DrivMan.nMan = 1
DrivMan.Man.Start.Velocity = 0
DrivMan.Man.Start.GearNo = 0
DrivMan.Man.Start.SteerAng = 0
DrivMan.Man.Start.LaneOffset = 0
DrivMan.Man.Start.OperatorActive = 1
DrivMan.Man.Start.OperatorState = drive
DrivMan.VhclOperator.Kind = IPGOperator 1
DrivMan.Man.0.nLongSteps = 1
DrivMan.Man.0.nLatSteps = 1
DrivMan.Man.0.CombinedSteps = 1
DrivMan.Man.0.LongStep.0.Info = Wait for Autoware - stationary fallback
DrivMan.Man.0.LongStep.0.Dyn = Manual
DrivMan.Man.0.LongStep.0.TimeLimit = 600
DrivMan.Man.0.LongStep.0.GearNo = 0
DrivMan.Man.0.LongStep.0.Gas = 0 0 0.2 abs
DrivMan.Man.0.LongStep.0.Brake = 0.5 0 0.2 abs
DrivMan.Man.0.LongStep.0.Clutch = 1 0 0.2 abs
DrivMan.Man.0.LatStep.0.Dyn = Driver 0
DrivMan.Man.0.LatStep.0.TimeLimit = 600
DrivMan.Man.0.LatStep.0.SteerAng = 0 0 0.2 abs
```

- The car starts standing still, with the brake on and the steering
  straight. Once Autoware sends `VehicleControl`, CMNode applies those
  commands in place of the manoeuvre's.
- `TimeLimit = 600` caps the run at 10 minutes. Raise it for long routes,
  such as the 10.8 km ADAS loop.
- If Autoware stops sending commands, the Autoware interface's watchdog
  brakes (`command_timeout_sec`, `timeout_brake` in the interface
  parameters).
- `DrivManTrigger` markers left on the road do no harm. They only start steps
  of the old manoeuvre.

Keep the `Driver.*` block of the working Autoware TestRun as well. It does
not drive with this manoeuvre, but avoids leftover settings from other
vehicles, such as motorcycle rider parameters.

---

## 9. Convert, build and run

### 9.1 Convert and build (one command)

```bash
cd $AW && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run autoware_carmaker_launch testrun_to_autoware <TestRun> --project $PRJ --build
#   add --force to regenerate, --base-profile <p> for unmapped vehicles,
#   --max-speed-kmh <v> to raise the 20 km/h cap
```

It prints the mode, the lanelet count, the validation result and any
**warnings**. Read them, especially vehicle size differences, cones it could
not place, and lap overlap.

Outputs:

| File | Content |
|---|---|
| `maps/<name>/lanelet2_map.osm` | lanelet map (local coordinates = CarMaker world) |
| `maps/<name>/pointcloud_map.pcd` | synthetic ground, cones and pylons (for display, not for localization) |
| `maps/<name>/map_projector_info.yaml` | local projector |
| `maps/<name>/scenario.yaml` | start and goal poses, waypoints, traffic objects, validation, warnings |
| `src/.../config/profiles/<name>.yaml` | launch profile (`auto_route: true`) |

`<name>` is the TestRun name in lower case, for example
`Slalom18m_Autoware` → `slalom18m_autoware`.

### 9.2 Run

Terminal 1, Autoware:

```bash
cd $AW && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch autoware_carmaker_launch carmaker.launch.py profile:=<name>
#   auto_engage:=true   engage autonomous mode automatically
#   rviz:=false         no RViz
#   camera_bridge:=false  only stops the Autoware camera relays
```

Terminal 2, CarMaker:

```bash
cd $PRJ && ./CMStart.sh 2
```

In Office, start the CMRosIF / CMNode launch action, load `<TestRun>` and
press **Start**. Once localization initializes, the route is set
automatically. Then engage in RViz's *Autoware* panel, unless you passed
`auto_engage:=true`.

---

## 10. Verification checklist

| Check | How |
|---|---|
| Converter validation passed | `validation.passed: true` in `scenario.yaml` |
| No important warnings | `warnings:` in `scenario.yaml` |
| Map loads | Autoware log: `Succeeded to load lanelet2_map` |
| Localization initialized | `ros2 topic echo /api/localization/initialization_state` → state 3 |
| Route set | log `Route set from scenario`; `ros2 topic echo /api/routing/state` → state 2 |
| Vehicle pose on the map | RViz: the car sits in the lane at the CarMaker start |
| Sensors arrive | `ros2 topic hz /sensing/camera/<name>/image_raw`, `/sensing/lidar/concatenated/pointcloud` |
| Sensor transforms correct | `ros2 run tf2_ros tf2_echo base_link <frame>` matches CarMaker `Sensor.<i>.pos` minus the offset |
| Autonomous available | `ros2 topic echo /api/operation_mode/state` → `is_autonomous_mode_available: true` |
| Drives | the car follows the corridor in IPGMovie; pylons stay untouched |

### Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `Reading junction ... failed, 'lengthOnLink' not equal to zero` | The TestRun was saved by CarMaker 13 or 14. Open and save it in CM15. |
| `No Autoware vehicle profile is known for CarMaker vehicle '...'` | Map the vehicle (section 3) or pass `--base-profile`. |
| `... already exists; use --force` | A previous conversion exists. Add `--force`. |
| Validation `route_found: false` | Start or goal is off the lanelets: check `Vehicle.StartPos` and `--goal-margin`. For a corridor, check the gate and cone layout. |
| `LidarRSI 0: Not enough free GPUSensors available` | Started without GPU sensor setup (standalone CLI). Start from Office. |
| A camera never appears in ROS | `Sensor.<i>.Active = 0`, or two cameras share a cluster (one per cluster), or a port clash between clusters. |
| Camera images arrive but Autoware drops them | Epoch tag mismatch after a restart. Restart the TestRun so odometry and cameras share the same epoch, and check `require_camera_epoch`. |
| Car sits about 0.5–0.9 m off its true position in RViz | `source_base_offset_x` ≠ `Wheel.rl.pos[x]` (section 3.2). |
| CarMaker driver steers against Autoware | The TestRun still uses `Driver` for the longitudinal step. Apply section 8. |
| Planning containers exit with SIGSEGV on Ctrl-C | Known shutdown crash, present before this work (`ISO_LANECHANGE.md`). It does not affect the run itself. |

---

## 11. Worked example: Slalom18m from the CarMaker Examples

This is what was done for `Slalom18m_Autoware` (profile `slalom18m_autoware`):

1. **Source:**
   `/opt/ipg/carmaker/linux64-15.0/Data/TestRun/Examples/VehicleDynamics/Handling/Slalom18m`.
   - Vehicle `Examples/DemoCar`, a 600 m straight road, IPGDriver at 58 km/h.
   - The slalom is 30 `DrvPylon` gates: the free gates are 9 m wide, and the
     slalom gates are 4.75 m wide at ±2.625 m.
2. **New TestRun:** `$PRJ/Data/TestRun/Slalom18m_Autoware`, built from
   `LaneChange_ISO_Autoware`, with the road, route, `Vehicle.Routing*` and
   `Vehicle.StartPos*` (5 m) from `Slalom18m`. As a result:
   - `Vehicle = Autoware/DemoCar_ISO` (sections 3.1 and 4.4);
   - the stationary fallback manoeuvre (section 8);
   - the description changed to say where it came from.
3. **Convert:**
   ```bash
   ros2 run autoware_carmaker_launch testrun_to_autoware Slalom18m_Autoware --project $PRJ --build
   ```
   Result: corridor mode with pylon boundaries, 30 lanelets, validation
   passed, and no warnings. The gate bounds match exactly. For example, at
   x = 318 m the right bound is +0.25 m and the left +5.0 m.
4. **Autoware check** (synthetic stationary pose): the map loaded,
   localization initialized after 10.5 s, and the route was set after 10.8 s
   (lanelet 1205 → 1292).
5. **Still to do:** a live CarMaker drive. To match the original's speed
   later, re-run with `--max-speed-kmh 58 --force` once slow runs are clean.

---

## 12. Quick reference: key cheat sheet

| Area | TestRun keys | Vehicle-file keys |
|---|---|---|
| Vehicle choice | `Vehicle`, `Trailer`, `Tire.0..3`, `VehicleLoad.*` | — |
| Start and route | `Vehicle.StartPos*`, `Vehicle.Routing*` | — |
| Manoeuvre and driver | `DrivMan.*`, `Driver.*` | — |
| Environment | `Env.*` | — |
| Road | `Road.*` or `Road.FName` | — |
| Traffic and cones | `Traffic.N`, `Traffic.<k>.*` | — |
| Geometry | — | `Wheel.<fl/fr/rl/rr>.pos`, `Vehicle.OuterSkin`, `Tire.*` |
| Sensors | — | `Sensor.Param.*`, `Sensor.<i>.*`, `SensorCluster.*`, `Autoware.SensorTFExternal` |

| Area | Autoware file |
|---|---|
| Vehicle size | `<vehicle>_description/config/vehicle_info.param.yaml` |
| Throttle and brake maps | `<vehicle>_launch/config/raw_vehicle_cmd_converter.param.yaml` (+ CSV) |
| Pose offset, steering, cameras on or off | `autoware_carmaker_interface/config/<x>.interface.param.yaml` |
| Camera topic list | `autoware_carmaker_interface/autoware_carmaker_interface/interface_node.py` (~line 220) |
| Sensor transforms | `<sensor_kit>_description/config/sensor_kit_calibration.yaml` + `urdf/sensors.xacro` |
| Which packages a run uses | `autoware_carmaker_launch/config/profiles/<name>.yaml` |
| CarMaker vehicle → profile | `autoware_carmaker_launch/autoware_carmaker_launch/testrun/profiles.py` (`VEHICLE_PROFILES`) |
