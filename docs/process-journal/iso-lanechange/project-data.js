window.PROCESS_JOURNAL = {
  "project": {
    "eyebrow": "ISO LANE-CHANGE / SESSION JOURNAL",
    "title": "ISO lane-change integration",
    "summary": "Reproducible commissioning record for copied LaneChange_ISO_Autoware + DemoCar_ISO under duplicate CarMaker project; isolated checks pass, full RSI remains blocked.",
    "status": "Built; isolated checks pass; full RSI blocked",
    "date": "2026-10-14",
    "testRun": "LaneChange_ISO_Autoware (copied from LaneChange_ISO_Camera_RSI)",
    "facts": [
      {
        "value": "Kept",
        "label": "Original vehicle and TestRun unchanged"
      },
      {
        "value": "CM15",
        "label": "Isolated copied CarMaker project; exact path in stage 1"
      },
      {
        "value": "2.537",
        "label": "Wheelbase in metres; rear-axle offset 0.863 m"
      },
      {
        "value": "7+3",
        "label": "Autoware and CarMaker packages built"
      },
      {
        "value": "46",
        "label": "Final unit suite result"
      },
      {
        "value": "91",
        "label": "DDS images relayed; 91 poses, 92 commands; rear x 450.863, watchdog brake 0.35"
      },
      {
        "value": "12s",
        "label": "NoRSI test: peak 0.701 m/s then stopped; no autonomy or real LiDAR"
      },
      {
        "value": "Pass",
        "label": "Simulator watchdog: controls stopped at 9 s, fallback observed, vehicle stopped"
      },
      {
        "value": "RSI",
        "label": "Blocked at 0 s by GPU sensor allocation error; root cause unknown"
      }
    ]
  },
  "architecture": [
    {
      "type": "source",
      "label": "01 / SOURCE",
      "title": "Duplicate CarMaker project",
      "detail": "FSAI_LTS_2026_CM15_Autoware copy holds DemoCar_ISO, LaneChange_ISO_Autoware and backed-up overlay; originals untouched."
    },
    {
      "type": "control",
      "label": "02 / BRIDGE",
      "title": "Autoware iso_lanechange_demo bridge",
      "detail": "Rear-axle URDF TF owner (Autoware.SensorTFExternal=1), shared TestRun epoch, stale-stop, manual gear/clutch, provisional steer -1/13.66."
    },
    {
      "type": "output",
      "label": "03 / TARGET",
      "title": "Local gate map + RViz engage gate",
      "detail": "Generated 10 km/h gate map and synthetic PCD for loading only; engage only after healthy data, route and trajectory at low speed."
    }
  ],
  "stages": [
    {
      "id": "protect-duplicate-overlay",
      "number": "01",
      "phase": "FOUNDATION",
      "title": "Protect originals, overlay duplicate only",
      "summary": "Original TestRun and DemoCar untouched; installer targets duplicate with backups and dry-run.",
      "action": "Dry-run overlay, verify diff in /tmp, then install into duplicate project only.",
      "implementation": [
        {
          "title": "Isolate the working project",
          "detail": "Keep LaneChange_ISO_Camera_RSI and Examples/DemoCar read-only; write DemoCar_ISO and LaneChange_ISO_Autoware copies plus _NoRSI variants only in the duplicate."
        },
        {
          "title": "Checked overlay install",
          "detail": "Installer resolves the complete patch chain, stages it under /tmp for inspection, writes backups beside the project, then copies."
        }
      ],
      "files": [
        "src/carmaker_integration/carmaker_overlay/install_overlay.py",
        "src/carmaker_integration/carmaker_overlay/iso_ros.patch",
        "src/carmaker_integration/carmaker_overlay/iso_shared_epoch.patch"
      ],
      "config": [
        {
          "key": "project",
          "value": "/home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware"
        },
        {
          "key": "protection",
          "value": "Original LaneChange_ISO_Camera_RSI and Examples/DemoCar untouched"
        },
        {
          "key": "overlay-safety",
          "value": "Backups beside project; full chain staged in /tmp before copy"
        }
      ],
      "commands": [
        "Example: /usr/bin/python3 src/carmaker_integration/carmaker_overlay/install_overlay.py --project /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware --dry-run",
        "Example: /usr/bin/python3 src/carmaker_integration/carmaker_overlay/install_overlay.py --project /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware"
      ],
      "verify": [
        "Duplicate holds DemoCar_ISO and LaneChange_ISO_Autoware copies",
        "Backups written beside duplicate project",
        "Dry-run tests pass before copy"
      ],
      "critical": true,
      "bugs": [
        {
          "symptom": "Optional installer patch could be silently skipped.",
          "diagnosis": "Partial-chain apply did not enforce completeness before copy.",
          "fix": "Resolve complete patch chain, stage and inspect it in /tmp, keep backups, then copy.",
          "result": "Dry-run tests pass; skipped-patch silent failure closed."
        }
      ]
    },
    {
      "id": "profile-geometry-sensors-map",
      "number": "02",
      "phase": "CONFIG",
      "title": "Pin profile geometry, sensors and local map",
      "summary": "Wheelbase 2.537 m, rear offset 0.863 m, square-pixel RGB, virtual IMU, 10 km/h gate map and synthetic PCD.",
      "action": "Use iso_lanechange_demo profile, demo vehicle/sensor-kit params, and generated local map only.",
      "implementation": [
        {
          "title": "Lock the demo profile",
          "detail": "Profile selects demo vehicle, demo sensor kit, demo_iso interface params and local ISO map path.",
          "code": [
            {
              "path": "src/carmaker_integration/autoware_carmaker_launch/config/profiles/iso_lanechange_demo.yaml",
              "language": "YAML",
              "startLine": 1,
              "content": "vehicle_model: carmaker_demo_vehicle\nsensor_model: carmaker_demo_sensor_kit\nvehicle_launch_package: carmaker_demo_vehicle_launch\ninterface_package: autoware_carmaker_interface\ninterface_params: demo_iso.interface.param.yaml\nconverter_params: raw_vehicle_cmd_converter.param.yaml\nmap_path: maps/iso_lanechange\ncamera_bridge: true"
            }
          ]
        },
        {
          "title": "Pin vehicle geometry",
          "detail": "DemoCar geometry gives wheelbase 2.537 m and rear overhang 0.863 m used as the rear-axle TF offset; front/rear RGB keep original mounts with square-pixel 1920x1080 horizontal 90 deg, virtual IMU, and Autoware.SensorTFExternal=1 so URDF owns TF.",
          "code": [
            {
              "path": "src/carmaker_integration/carmaker_demo_vehicle_description/config/vehicle_info.param.yaml",
              "language": "YAML",
              "startLine": 4,
              "content": "    wheel_radius: 0.318\n    wheel_width: 0.195\n    wheel_base: 2.537\n    wheel_tread: 1.578\n    front_overhang: 0.880\n    rear_overhang: 0.863"
            }
          ]
        }
      ],
      "files": [
        "src/carmaker_integration/autoware_carmaker_launch/config/profiles/iso_lanechange_demo.yaml",
        "src/carmaker_integration/carmaker_demo_vehicle_description/config/vehicle_info.param.yaml",
        "src/carmaker_integration/carmaker_demo_sensor_kit_description/config/sensor_kit_calibration.yaml",
        "maps/iso_lanechange/"
      ],
      "config": [
        {
          "key": "wheelbase / rear-offset",
          "value": "2.537 m / 0.863 m"
        },
        {
          "key": "cameras / IMU / TF",
          "value": "Front/rear RGB original mounts, square-pixel 1920x1080 horizontal 90 deg, virtual IMU, Autoware.SensorTFExternal=1"
        },
        {
          "key": "map / PCD scope",
          "value": "Local gate map 10 km/h; synthetic PCD for loading only, NOT NDT-grade or 90 km/h benchmark"
        },
        {
          "key": "steering / pedals",
          "value": "Sign -1 ratio 13.66 and pedal maps PROVISIONAL, calibration required"
        }
      ],
      "commands": [
        "Example: ros2 launch autoware_carmaker_launch carmaker.launch.py profile:=iso_lanechange_demo",
        "Observed (~45s startup): Autoware map and robot model loaded with no sensors; readiness diagnostics waited for sensor data and route. Autonomous engage was not attempted."
      ],
      "verify": [
        "xacro and ISO map load",
        "~45 s startup shows map and robot loaded",
        "No claim of 90 km/h, NDT localization, or autonomous lane change"
      ],
      "critical": false,
      "bugs": [
        {
          "symptom": "Lateral setting Dyn=Manual was invalid for the copied vehicle.",
          "diagnosis": "CarMaker explicitly reported Unknown LatDyn 'Manual'; that lateral dynamics token is unsupported.",
          "fix": "Set DrivMan.Man.0.LatStep.0.Dyn = Driver 0 in the copied TestRun.",
          "result": "NoRSI 12 s vehicle check passed."
        }
      ]
    },
    {
      "id": "epoch-bridge-build",
      "number": "03",
      "phase": "BUILD",
      "title": "Build epoch bridge and both workspaces",
      "summary": "Shared TestRun epoch, stale-stop, manual gear/clutch; seven Autoware and three CM packages built, 46 tests pass.",
      "action": "Build selected Autoware and CM packages in their own workspaces after any msg change.",
      "implementation": [
        {
          "title": "Pin the epoch and actuation contract",
          "detail": "Raw odom and cameras share the same TestRun epoch IDs; the Humble bridge strips the tag, rejects retired epochs, stops on stale odom/controls, and shares manual gear/clutch fields across both workspaces.",
          "code": [
            {
              "path": "src/carmaker_integration/autoware_carmaker_interface/config/demo_iso.interface.param.yaml",
              "language": "YAML",
              "startLine": 7,
              "content": "    source_base_offset_x: 0.863\n    source_base_offset_y: 0.0\n    source_base_offset_z: 0.0\n    # CarMaker GenAngle handwheel -> tire conversion. Initial small-angle\n    # McPherson value; verify sign and response with the supplied smoke test.\n    steering_command_sign: -1.0\n    steering_command_ratio: 13.66\n    steering_feedback_sign: -1.0\n    steering_feedback_ratio: 13.66"
            }
          ]
        },
        {
          "title": "Build both sides",
          "detail": "Build seven Autoware packages with symlink install, then three CM-side packages with CarMaker 15.0 flags; rebuild vehiclecontrol_msgs in both workspaces after msg edits."
        }
      ],
      "files": [
        "src/carmaker_integration/autoware_carmaker_interface/config/demo_iso.interface.param.yaml",
        "src/carmaker_integration/autoware_carmaker_interface/autoware_carmaker_interface/interface_node.py",
        "src/carmaker_integration/vehiclecontrol_msgs/msg/VehicleControl.msg"
      ],
      "config": [
        {
          "key": "epoch",
          "value": "Same TestRun epoch IDs on raw odom/cameras, removed by bridge"
        },
        {
          "key": "safety",
          "value": "Stale odom/controls stop; timeouts 0.5 s, watchdog brake 0.35"
        },
        {
          "key": "transmission",
          "value": "Manual gear/clutch fields shared by both workspaces"
        }
      ],
      "commands": [
        "Example: PATH=/usr/bin:/bin:$PATH /usr/bin/colcon build --symlink-install --base-paths src/carmaker_integration --packages-select vehiclecontrol_msgs autoware_carmaker_interface autoware_carmaker_launch carmaker_demo_vehicle_description carmaker_demo_vehicle_launch carmaker_demo_sensor_kit_description carmaker_demo_sensor_kit_launch",
        "Example: PATH=/usr/bin:/bin:$PATH /usr/bin/colcon build --packages-select vehiclecontrol_msgs hellocm_cmnode carmaker_rsds_client --cmake-args -DCARMAKER_VER:STRING=15.0 -DCARMAKER_DIR:PATH=/opt/ipg/carmaker/linux64-15.0",
        "Observed (46 passed, cwd /home/klee/autoware.testvehicle): source /opt/ros/humble/setup.bash; source install/setup.bash; PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/carmaker_integration/autoware_carmaker_interface:src/carmaker_integration/autoware_carmaker_launch:${PYTHONPATH} /usr/bin/python3 -m pytest -q src/carmaker_integration/autoware_carmaker_interface/test src/carmaker_integration/autoware_carmaker_launch/test"
      ],
      "verify": [
        "Seven Autoware and three CM packages BUILT",
        "Final unit suite reports 46 passed",
        "vehiclecontrol_msgs rebuilt in both workspaces"
      ],
      "critical": true,
      "bugs": [
        {
          "symptom": "Two-argument camera callback failed under Humble.",
          "diagnosis": "Subscription callback signature did not match Humble relay API.",
          "fix": "Switched to one-argument callback form.",
          "result": "Real-DDS synthetic camera relay passed."
        }
      ]
    },
    {
      "id": "isolated-checks-pass",
      "number": "04",
      "phase": "VERIFY",
      "title": "Run isolated synthetic and NoRSI checks",
      "summary": "Real-DDS synthetic PASSED 91/91/92 and NoRSI 12 s PASSED 0.701 m/s; no autonomy or real LiDAR claimed.",
      "action": "Run DDS smoke with synthetic inputs and the separate NoRSI drive check; record counts as observed.",
      "implementation": [
        {
          "title": "Real-DDS synthetic bridge check",
          "detail": "Synthetic rear-axle poses, image relays and manual gear/clutch commands on an isolated domain; stale commands brake; final run PASSED with 91 poses, 91 images and 92 commands at rear x 450.863."
        },
        {
          "title": "NoRSI vehicle-path check",
          "detail": "GPU sensors disabled via explicit _NoRSI copies; low-throttle commissioning run ends with braking, reached 0.701 m/s peak and stopped after about 0.2 m of travel. Empty clouds expected; this is basic actuator response, not calibrated driving, autonomy or real LiDAR."
        },
        {
          "title": "Simulator-side watchdog check",
          "detail": "Separate 12 s check deliberately stopped controls at 9 s. CarMaker logged 'Autoware VC watchdog: gas=0 brake=0.5 gear=0 clutch=1'. Peak speed 0.693 m/s, final speed -0.000080 m/s, passed. Report /tmp/iso-watchdog-control-report.json; log /tmp/iso-carmaker-smoke-d03dcp_g/carmaker.log."
        }
      ],
      "files": [
        "src/carmaker_integration/autoware_carmaker_interface/test/smoke_interface.py",
        "src/carmaker_integration/autoware_carmaker_interface/test/test_iso_callbacks.py",
        "src/carmaker_integration/carmaker_overlay/smoke_runtime.py"
      ],
      "config": [
        {
          "key": "DDS result",
          "value": "PASSED 91 poses / 91 images / 92 commands, rear x 450.863, watchdog brake 0.35"
        },
        {
          "key": "NoRSI result",
          "value": "12 s PASSED, peak 0.701 m/s, stopped; NOT autonomous lane change"
        },
        {
          "key": "LiDAR scope",
          "value": "Empty clouds in these checks; no real-LiDAR claim"
        },
        {
          "key": "simulator watchdog",
          "value": "Observed safe-stop diagnostic, gas 0 / brake 0.5 / gear 0 / clutch 1 after control messages ceased"
        }
      ],
      "commands": [
        "Example: ROS_DOMAIN_ID=55 /usr/bin/python3 src/carmaker_integration/autoware_carmaker_interface/test/smoke_interface.py",
        "Example: ROS_DOMAIN_ID=54 /usr/bin/python3 src/carmaker_integration/carmaker_overlay/smoke_runtime.py --project /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware --no-rsi --drive-check --seconds 12 --report /tmp/iso-vehicle-control-report.json",
        "Observed (passed, cwd duplicated CarMaker project): source ros_setup.bash 2; source /home/klee/autoware.testvehicle/install/setup.bash; ROS_DOMAIN_ID=54 /usr/bin/python3 /home/klee/autoware.testvehicle/src/carmaker_integration/carmaker_overlay/smoke_runtime.py --project /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware --seconds 12 --no-rsi --drive-check --watchdog-check --report /tmp/iso-watchdog-control-report.json",
        "Observed (synthetic): real-DDS final PASSED 91 poses / 91 images / 92 commands"
      ],
      "verify": [
        "Real-DDS synthetic final PASSED as counted above",
        "NoRSI 12 s vehicle test PASSED with safe stop",
        "Separate simulator watchdog check PASSED with explicit fallback diagnostic and stopped vehicle",
        "No autonomous lane-change or real-LiDAR result claimed"
      ],
      "critical": false,
      "bugs": []
    },
    {
      "id": "rsi-blocked-launch-procedure",
      "number": "05",
      "phase": "OPERATE",
      "title": "Follow gated launch; RSI full run blocked",
      "summary": "Operational start order documented; 8 s RSI aborted at 0 s on GPU-sensor error; shutdown segfault remains open.",
      "action": "Start Autoware profile, CarMaker office, and hellocm service in order; engage only after healthy data, route and trajectory, low speed first.",
      "implementation": [
        {
          "title": "Ordered start",
          "detail": "Source Autoware setup, launch iso_lanechange_demo profile, start CarMaker with CMStart.sh 2, source ros_setup.bash 2, then start hellocm service only if needed."
        },
        {
          "title": "Gated engage and known blocks",
          "detail": "Set RViz 2D Goal Pose ahead on lane centerline (local map x ~650, y 0); require route plus live planning trajectory before any motion, low speed first. Full 8 s RSI run aborted at 0 s with LidarRSI 0 GPU-sensor error on healthy RTX 5060 Ti 16 GB; license/resource cause UNKNOWN."
        }
      ],
      "files": [
        "src/carmaker_integration/autoware_carmaker_launch/launch/carmaker.launch.py",
        "src/carmaker_integration/ISO_LANECHANGE.md"
      ],
      "config": [
        {
          "key": "launch-profile",
          "value": "ros2 launch autoware_carmaker_launch carmaker.launch.py profile:=iso_lanechange_demo"
        },
        {
          "key": "carmaker-start",
          "value": "./CMStart.sh 2; source ros_setup.bash 2 then hellocm service if needed"
        },
        {
          "key": "engage-gate",
          "value": "Healthy data plus route plus trajectory; 2D Goal Pose x ~650 y 0; low speed first"
        },
        {
          "key": "rsi-block",
          "value": "LidarRSI 0: Not enough free GPUSensors available at 0 s; RTX 5060 Ti 16 GB healthy; cause UNKNOWN"
        }
      ],
      "commands": [
        "Example: ros2 launch autoware_carmaker_launch carmaker.launch.py profile:=iso_lanechange_demo",
        "Example: ./CMStart.sh 2",
        "Example: source ros_setup.bash 2",
        "Example: ros2 launch hellocm hellocm.launch.py use_sim_time:=True rviz2:=False",
        "Observed (blocked): 8 s RSI run aborted at 0 s with LidarRSI GPU-sensor error; hardware healthy"
      ],
      "verify": [
        "No completed autonomous lane change claimed",
        "RSI root cause remains UNKNOWN",
        "SIGINT shutdown motion-planning container -11 remains unresolved"
      ],
      "critical": true,
      "bugs": [
        {
          "symptom": "Copied RSI TestRun aborted at 0 s: LidarRSI 0: Not enough free GPUSensors available.",
          "diagnosis": "Unresolved allocation failure; healthy GPU and no competing CarMaker process were observed, but license vs rendering/compatibility cause is not confirmed.",
          "fix": "No RSI fix claimed. Preserve main sensors and use separate NoRSI copies for bounded actuator commissioning.",
          "result": "NoRSI controls and watchdog checked; real RSI sensor data and autonomous lane change remain blocked."
        },
        {
          "symptom": "SIGINT shutdown segfaults motion-planning container with -11.",
          "diagnosis": "Unresolved; observed during shutdown only. Effect on driving has not been assessed.",
          "fix": "No fix claimed; left open for rerun.",
          "result": "Still unresolved."
        }
      ]
    }
  ],
  "restart": [
    {
      "state": "CLEAN BASELINE",
      "detail": "Originals untouched; overlay backups retained; duplicate built with profile and map loaded"
    },
    {
      "state": "GATED ENGAGE",
      "detail": "Move only after healthy sensors, route received and live trajectory; low speed first"
    },
    {
      "state": "FAULT HOLD",
      "detail": "On stale data, retired epoch or RSI GPU-sensor fault, hold motion and re-verify",
      "alert": true
    },
    {
      "state": "ISOLATED PASS",
      "detail": "Synthetic DDS and NoRSI checks pass; full RSI still blocked",
      "success": true
    }
  ]
};
