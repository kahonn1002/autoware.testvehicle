window.PROCESS_JOURNAL = {
  project: {
    eyebrow: "CARMAKER 15 × AUTOWARE / COMPLETE",
    title: "From simulator sensor output to autonomous vehicle control",
    summary:
      "A visual trace of how two isolated workspaces became one reliable closed-loop system—and how each blocker was diagnosed instead of bypassed.",
    status: "Working end-to-end",
    date: "2026-07-29",
    testRun: "FS_autonomous_Sprint_LTS",
    facts: [
      { value: "13", label: "implementation stages" },
      { value: "12", label: "debugged failure classes" },
      { value: "7", label: "isolated ROS packages" },
      { value: "0", label: "changes to main Autoware" }
    ]
  },

  architecture: [
    {
      type: "source",
      label: "01 / SIMULATOR",
      title: "CarMaker 15",
      detail: "Clock · odometry · LiDAR · IMU · steering"
    },
    {
      type: "bridge",
      label: "02 / CONTRACT",
      title: "CMROSIF + CMNode",
      detail: "PointXYZIRC · VehicleControl · monotonic clock"
    },
    {
      type: "logic",
      label: "03 / ADAPTER",
      title: "Autoware interface",
      detail: "Localization · status · watchdog · reset safety"
    },
    {
      type: "plan",
      label: "04 / AUTONOMY",
      title: "Perception → planning",
      detail: "Occupancy · route · moving trajectory"
    },
    {
      type: "control",
      label: "05 / CLOSED LOOP",
      title: "Control → vehicle",
      detail: "Actuation · gear · gas · brake · steering"
    }
  ],

  stages: [
    {
      id: "isolation",
      number: "01",
      phase: "FOUNDATION",
      title: "Isolate both codebases",
      summary: "Protect the production checkouts before integration work begins.",
      action: "Use autoware.testvehicle and a duplicated CM15 project as the only writable integration targets.",
      implementation: [
        {
          title: "Freeze the source-of-record paths",
          detail: "Treat /home/klee/autoware and FSAI_LTS_2026_CM15 as read-only references. Do not build experimental packages or modify vehicle assets in either source."
        },
        {
          title: "Prepare the isolated Autoware workspace",
          detail: "Use /home/klee/autoware.testvehicle as the ROS workspace and create src/carmaker_integration for all new interface, vehicle, sensor, message, and launch packages."
        },
        {
          title: "Pin the matching launcher",
          detail: "In the isolated repositories/autoware.repos, replace launcher/autoware_launch 0.50.0 with the test-vehicle launcher fork pinned to commit 8ed295fd7a3f0be4c89b593e0a31f4631904b1f0.",
          code: [
            {
              path: "repositories/autoware.repos",
              language: "YAML",
              startLine: 102,
              content: "# launcher\nlauncher/autoware_launch.tutorial_vehicle:\n  type: git\n  url: https://github.com/kleolee/autoware_launch.tutorial_vehicle\n  version: 8ed295fd7a3f0be4c89b593e0a31f4631904b1f0"
            }
          ]
        },
        {
          title: "Replicate the CarMaker project",
          detail: "Copy the full CM15 project tree to FSAI_LTS_2026_CM15_Autoware so Data, GUI, src, ros, build scripts, TestRuns, and vehicle infofiles remain a self-contained CarMaker project."
        },
        {
          title: "Use ROS Humble's Python",
          detail: "Prepend /usr/bin:/bin during builds so a Conda environment cannot replace the Python 3.10 interpreter and ROS packages expected by Humble."
        }
      ],
      files: [
        "repositories/autoware.repos",
        "src/carmaker_integration/",
        "FSAI_LTS_2026_CM15_Autoware/AUTOWARE_INTEGRATION.md"
      ],
      config: [
        { key: "Autoware runtime", value: "/home/klee/autoware.testvehicle" },
        { key: "CarMaker runtime", value: "FSAI_LTS_2026_CM15_Autoware" },
        { key: "launcher commit", value: "8ed295fd7a3f0be4c89b593e0a31f4631904b1f0" }
      ],
      commands: [
        "rsync -a /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15/ /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware/",
        "cd /home/klee/autoware.testvehicle\nsource /opt/ros/humble/setup.bash"
      ],
      verify: [
        "The duplicated CM project opens independently in CarMaker 15",
        "The integration directory exists only in the isolated Autoware workspace",
        "The main /home/klee/autoware checkout has no integration changes"
      ],
      critical: true,
      bugs: [
        {
          symptom: "The user explicitly required the main Autoware repository to remain untouched.",
          diagnosis: "Building directly in the primary checkout would mix experimental packages, generated maps, and launch overrides with the source of record.",
          fix: "Created a two-workspace boundary and kept every integration artifact inside the isolated copies.",
          result: "The original Autoware and original CM15 project remain independent of the runtime integration."
        }
      ]
    },
    {
      id: "contracts",
      number: "02",
      phase: "PORT",
      title: "Port CMROSIF to exact ROS contracts",
      summary: "Replace example-level bridging with deterministic interfaces.",
      action: "Build CM15-side publishers and the shared VehicleControl subscriber at the correct CarMaker callback hook.",
      implementation: [
        {
          title: "Create the shared ROS message package",
          detail: "Create vehiclecontrol_msgs with rosidl_default_generators. Define use_vc, selector_ctrl, gas, brake, steer_ang, steer_ang_vel, and steer_ang_acc.",
          code: [
            {
              path: "vehiclecontrol_msgs/msg/VehicleControl.msg",
              language: "ROS MSG",
              startLine: 1,
              content: "# CarMaker VehicleControl input. All values are finite and clamped by the bridge.\nbool use_vc\nint8 selector_ctrl\nfloat64 gas\nfloat64 brake\nfloat64 steer_ang\nfloat64 steer_ang_vel\nfloat64 steer_ang_acc"
            }
          ]
        },
        {
          title: "Install the message on both sides",
          detail: "Place the same vehiclecontrol_msgs package in the isolated Autoware workspace and in CarMaker's ros/ros2_ws/src so publisher and subscriber compile against one type definition.",
          code: [
            {
              path: "vehiclecontrol_msgs/CMakeLists.txt",
              language: "CMAKE",
              startLine: 4,
              content: "find_package(ament_cmake REQUIRED)\nfind_package(rosidl_default_generators REQUIRED)\n\nrosidl_generate_interfaces(${PROJECT_NAME}\n  \"msg/VehicleControl.msg\"\n)\n\nament_export_dependencies(rosidl_default_runtime)"
            }
          ]
        },
        {
          title: "Declare CMNode dependencies",
          detail: "Add find_package(vehiclecontrol_msgs REQUIRED), nav_msgs, sensor_msgs, angles and TF dependencies to hellocm_cmnode/CMakeLists.txt and package.xml.",
          code: [
            {
              path: "ros/ros2_ws/src/hellocm_cmnode/CMakeLists.txt",
              language: "CMAKE",
              startLine: 21,
              content: "# find dependencies\nfind_package(ament_cmake REQUIRED)\nfind_package(angles REQUIRED)\nfind_package(rclcpp REQUIRED)\nfind_package(rclpy REQUIRED)\nfind_package(std_msgs REQUIRED)\nfind_package(cmrosutils REQUIRED)\nfind_package(hellocm_msgs REQUIRED)\nfind_package(vehiclecontrol_msgs REQUIRED)\nfind_package(camera_msgs REQUIRED)\nfind_package(tf2 REQUIRED)\nfind_package(tf2_ros REQUIRED)\nfind_package(tf2_geometry_msgs REQUIRED)\nfind_package(visualization_msgs REQUIRED)\nfind_package(sensor_msgs REQUIRED)\nfind_package(nav_msgs REQUIRED)"
            }
          ]
        },
        {
          title: "Create the VehicleControl subscriber job",
          detail: "Register a RosSubscriber on relative topic VehicleControl inside the /carmaker namespace, run every CarMaker cycle, skip the first cycle, and copy the message into CMNode.VC.",
          code: [
            {
              path: "hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp",
              language: "C++",
              startLine: 913,
              content: "    { /* set up VehicleControl subscriber + job */\n      std::string topic = \"VehicleControl\";\n      bool synchronize = (sync_mode_ == SyncMode::kTopic);\n      CMJob::JobType job_type =\n          synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;\n\n      typedef CMJob::RosSubscriber<vehiclecontrol_msgs::msg::VehicleControl> VC;\n      auto job = std::make_shared<VC>(job_type, synchronize, nhp_, topic);\n      job->setCycleTime(static_cast<unsigned long>(1));\n      job->skipFirstCycles(1);\n      job->registerCallback(&CMNodeHelloCM::vehicleControlCallback, this);\n      scheduler_.addJob(job);\n    } // set up VehicleControl subscriber + job"
            },
            {
              path: "hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp",
              language: "C++",
              startLine: 165,
              content: "void CMNodeHelloCM::vehicleControlCallback(vehiclecontrol_msgs::msg::VehicleControl::ConstSharedPtr msg) {\n\n    CMNode.VC.use_vc          = msg->use_vc;\n    CMNode.VC.selector_ctrl   = msg->selector_ctrl;\n    CMNode.VC.gas             = msg->gas;\n    CMNode.VC.brake           = msg->brake;\n    CMNode.VC.steer_ang       = msg->steer_ang;\n    CMNode.VC.steer_ang_vel   = msg->steer_ang_vel;\n    CMNode.VC.steer_ang_acc   = msg->steer_ang_acc;"
            }
          ]
        },
        {
          title: "Apply commands at the correct hook",
          detail: "Call CMCppIFLoader_vehicleControlCalc from User_VehicleControl_Calc, after VehicleControl_Calc. Fall back to DrivMan values unless use_vc=1.",
          code: [
            {
              path: "src/User.cpp",
              language: "C++",
              startLine: 553,
              content: "int\nUser_VehicleControl_Calc(double dt)\n{\n    /* Rely on the Vehicle Operator within DrivMan module to get\n       the vehicle in driving state using the IPG's\n       PowerTrain Control model 'Generic' or similar */\n    if (Vehicle.OperationState != OperState_Driving) {\n        return 0;\n    }\n\n#if defined(WITH_CMCPPIF)\n    CMCppIFLoader_vehicleControlCalc(&dt);\n#endif"
            },
            {
              path: "hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp",
              language: "C++",
              startLine: 263,
              content: "    if (CMNode.VC.use_vc == 1) {\n            \n        /* Assign ROS Topic to CM Vehicle Control Quantities */\n        /* -->Automatic transmission PRNDL selection */\n        VehicleControl.SelectorCtrl     = CMNode.VC.selector_ctrl;\n\n        /* -->Pedal Positions (add clutch if needed for manual) */\n        VehicleControl.Gas              = CMNode.VC.gas;\n        VehicleControl.Brake            = CMNode.VC.brake;\n\n        /* -->Steer by angle inputs (add steer torque if using GenTorque model) */\n        VehicleControl.Steering.Ang     = CMNode.VC.steer_ang;\n        VehicleControl.Steering.AngVel  = CMNode.VC.steer_ang_vel;\n        VehicleControl.Steering.AngAcc  = CMNode.VC.steer_ang_acc;\n\n    }"
            }
          ]
        },
        {
          title: "Reset authority at simulation start",
          detail: "When SimCore.Time is below 0.001 s, clear use_vc so a stale ROS command from a previous TestRun cannot immediately control the new run.",
          code: [
            {
              path: "hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp",
              language: "C++",
              startLine: 259,
              content: "    if (SimCore.Time < 0.001) {\n        CMNode.VC.use_vc = 0; //Reset back to DM at beginning of simulation...\n    }"
            }
          ]
        },
        {
          title: "Publish the simulator feedback set",
          detail: "Register 100 Hz jobs for odometry, IMU and measured steering, plus the LidarRSI publisher at its scan cadence. Use the /carmaker node namespace so relative names become the documented contract.",
          code: [
            {
              path: "hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp",
              language: "C++",
              startLine: 948,
              content: "    { /* set up odometry publisher + job\n       * Topic: /odometry  |  100 Hz  |  nav_msgs/Odometry */\n      typedef CMJob::RosPublisher<nav_msgs::msg::Odometry> OdometryPub;\n      auto job = std::make_shared<OdometryPub>(nhp_, \"odometry\");\n      job->setCycleTime(static_cast<unsigned long>(10)); // 10 ms = 100 Hz\n      job->skipFirstCycles(1);\n      job->registerCallback(&CMNodeHelloCM::odometryFillMsg, this);\n      scheduler_.addJob(job);\n    } // set up odometry publisher + job"
            },
            {
              path: "hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp",
              language: "C++",
              startLine: 958,
              content: "    { /* set up measured steering angle publisher + job */\n      typedef CMJob::RosPublisher<std_msgs::msg::Float64> SteeringPub;\n      auto job = std::make_shared<SteeringPub>(nhp_, \"steering_angle\");\n      job->setCycleTime(static_cast<unsigned long>(10)); // 10 ms = 100 Hz\n      job->skipFirstCycles(1);\n      job->registerCallback(&CMNodeHelloCM::steeringFillMsg, this);\n      scheduler_.addJob(job);\n    }"
            }
          ]
        }
      ],
      files: [
        "vehiclecontrol_msgs/msg/VehicleControl.msg",
        "ros/ros2_ws/src/hellocm_cmnode/CMakeLists.txt",
        "ros/ros2_ws/src/hellocm_cmnode/package.xml",
        "ros/ros2_ws/src/hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp",
        "src/User.cpp"
      ],
      config: [
        { key: "subscriber topic", value: "/carmaker/VehicleControl" },
        { key: "control authority", value: "use_vc == 1" },
        { key: "CarMaker hook", value: "User_VehicleControl_Calc()" }
      ],
      commands: [
        "cd /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware\nPATH=/usr/bin:/bin:$PATH ./build.sh 2",
        "source ros_setup.bash 2\nros2 interface show vehiclecontrol_msgs/msg/VehicleControl"
      ],
      verify: [
        "CMNode shared library links vehiclecontrol_msgs",
        "ros2 topic info /carmaker/VehicleControl shows one publisher and one subscriber",
        "With use_vc=false, DrivMan retains authority; with true, ROS values reach VehicleControl"
      ],
      critical: true,
      bugs: [
        {
          symptom: "The initial example bridge relayed some sensors but did not satisfy Autoware localization, status, reset, and actuation contracts.",
          diagnosis: "Runtime type discovery and generic relays are insufficient for safety-gated Autoware nodes.",
          fix: "Created dedicated interface packages and an explicit VehicleControl message shared by both workspaces.",
          result: "Simulator output and Autoware input now meet a stable, inspectable contract."
        }
      ]
    },
    {
      id: "vehicle-interface",
      number: "03",
      phase: "ADAPTER",
      title: "Define the Autoware vehicle and sensor kit",
      summary: "Describe the FSAI geometry, sensor transforms, command conversion, and simulator adapter as installable ROS packages.",
      action: "Create the vehicle-description, vehicle-launch, sensor-description, sensor-launch, and interface packages expected by autoware_launch.",
      implementation: [
        {
          title: "Create the vehicle-description package",
          detail: "Use ament_cmake and install config/ plus urdf/. vehicle_info.param.yaml defines the dimensions consumed by Autoware; vehicle.xacro provides base_link visualization and collision geometry.",
          code: [
            {
              path: "carmaker_fsai_vehicle_description/CMakeLists.txt",
              language: "CMAKE",
              startLine: 1,
              content: "cmake_minimum_required(VERSION 3.14)\nproject(carmaker_fsai_vehicle_description)\nfind_package(ament_cmake REQUIRED)\ninstall(DIRECTORY config urdf DESTINATION share/${PROJECT_NAME})\nament_package()"
            },
            {
              path: "carmaker_fsai_vehicle_description/urdf/vehicle.xacro",
              language: "XACRO",
              startLine: 2,
              content: "<robot xmlns:xacro=\"http://ros.org/wiki/xacro\">\n  <xacro:property name=\"vehicle_info\" value=\"${xacro.load_yaml('$(find carmaker_fsai_vehicle_description)/config/vehicle_info.param.yaml')}\"/>\n  <link name=\"base_link\">\n    <visual>\n      <origin xyz=\"0.842 0 0.5\" rpy=\"0 0 0\"/>\n      <geometry><box size=\"2.650 1.400 1.000\"/></geometry>\n      <material name=\"fsai_blue\"><color rgba=\"0.05 0.25 0.8 0.7\"/></material>\n    </visual>\n    <collision>\n      <origin xyz=\"0.842 0 0.5\" rpy=\"0 0 0\"/>\n      <geometry><box size=\"2.650 1.400 1.000\"/></geometry>\n    </collision>\n  </link>\n</robot>"
            }
          ]
        },
        {
          title: "Enter FSAI geometry",
          detail: "Define wheel radius 0.2576 m, width 0.185 m, wheelbase 1.600 m, tread 1.200 m, front/rear overhang 0.567/0.483 m, side overhang 0.100 m, height 1.000 m, and max steering 0.70 rad.",
          code: [
            {
              path: "carmaker_fsai_vehicle_description/config/vehicle_info.param.yaml",
              language: "YAML",
              startLine: 1,
              content: "/**:\n  ros__parameters:\n    wheel_radius: 0.2576\n    wheel_width: 0.185\n    wheel_base: 1.600\n    wheel_tread: 1.200\n    front_overhang: 0.567\n    rear_overhang: 0.483\n    left_overhang: 0.100\n    right_overhang: 0.100\n    vehicle_height: 1.000\n    max_steer_angle: 0.70"
            }
          ]
        },
        {
          title: "Create the sensor-kit description",
          detail: "Use fixed Xacro joints from base_link to carmaker_lidar and carmaker_imu. Match the CarMaker vehicle infofile positions exactly: LiDAR (1.7, 0, 0.57), IMU (1.3, 0, 0.3).",
          code: [
            {
              path: "carmaker_sensor_kit_description/config/sensor_kit_calibration.yaml",
              language: "YAML",
              startLine: 1,
              content: "base_link:\n  carmaker_lidar:\n    x: 1.7\n    y: 0.0\n    z: 0.57\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n  carmaker_imu:\n    x: 1.3\n    y: 0.0\n    z: 0.3\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0"
            }
          ]
        },
        {
          title: "Create acceleration and brake maps",
          detail: "Define CSV lookup tables by pedal and speed from 0–25 m/s. The initial acceleration map maps full pedal to 4.0 m/s² at zero speed and 1.5 m/s² at 25 m/s; braking maps full pedal to -8.0 m/s².",
          code: [
            {
              path: "carmaker_fsai_vehicle_launch/data/accel_map.csv",
              language: "CSV",
              startLine: 1,
              content: "default,0,5,10,15,20,25\n0.0,0.0,0.0,0.0,0.0,0.0,0.0\n0.1,0.4,0.35,0.3,0.25,0.2,0.15\n0.2,0.8,0.7,0.6,0.5,0.4,0.3\n0.3,1.2,1.05,0.9,0.75,0.6,0.45\n0.4,1.6,1.4,1.2,1.0,0.8,0.6\n0.5,2.0,1.75,1.5,1.25,1.0,0.75\n0.6,2.4,2.1,1.8,1.5,1.2,0.9\n0.8,3.2,2.8,2.4,2.0,1.6,1.2\n1.0,4.0,3.5,3.0,2.5,2.0,1.5"
            },
            {
              path: "carmaker_fsai_vehicle_launch/data/brake_map.csv",
              language: "CSV",
              startLine: 1,
              content: "default,0,5,10,15,20,25\n0.0,0.0,0.0,0.0,0.0,0.0,0.0\n0.1,-0.8,-0.8,-0.8,-0.8,-0.8,-0.8\n0.2,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6\n0.3,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4\n0.4,-3.2,-3.2,-3.2,-3.2,-3.2,-3.2\n0.5,-4.0,-4.0,-4.0,-4.0,-4.0,-4.0\n0.6,-4.8,-4.8,-4.8,-4.8,-4.8,-4.8\n0.8,-6.4,-6.4,-6.4,-6.4,-6.4,-6.4\n1.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0"
            }
          ]
        },
        {
          title: "Configure raw command conversion",
          detail: "Enable acceleration and brake conversion, leave steering conversion disabled because Autoware already supplies tire angle, clamp throttle/brake to 1.0 and steering to ±0.70 rad.",
          code: [
            {
              path: "carmaker_fsai_vehicle_launch/config/raw_vehicle_cmd_converter.param.yaml",
              language: "YAML",
              startLine: 3,
              content: "    csv_path_accel_map: $(find-pkg-share carmaker_fsai_vehicle_launch)/data/accel_map.csv\n    csv_path_brake_map: $(find-pkg-share carmaker_fsai_vehicle_launch)/data/brake_map.csv\n    convert_accel_cmd: true\n    convert_brake_cmd: true\n    convert_steer_cmd: false\n    use_steer_ff: false\n    use_steer_fb: false\n    is_debugging: false\n    max_throttle: 1.0\n    max_brake: 1.0\n    max_steer: 0.70\n    min_steer: -0.70"
            }
          ]
        },
        {
          title: "Wire the converter topics",
          detail: "Remap /control/command/control_cmd plus odometry, steering and actuation status into autoware_raw_vehicle_cmd_converter; publish /control/command/actuation_cmd.",
          code: [
            {
              path: "carmaker_fsai_vehicle_launch/launch/vehicle_interface.launch.xml",
              language: "XML",
              startLine: 6,
              content: "  <node pkg=\"autoware_raw_vehicle_cmd_converter\" exec=\"autoware_raw_vehicle_cmd_converter_node\" name=\"raw_vehicle_cmd_converter\" output=\"screen\">\n    <param from=\"$(var raw_vehicle_cmd_converter_param_path)\" allow_substs=\"true\"/>\n    <remap from=\"~/input/control_cmd\" to=\"/control/command/control_cmd\"/>\n    <remap from=\"~/input/odometry\" to=\"/localization/kinematic_state\"/>\n    <remap from=\"~/input/steering\" to=\"/vehicle/status/steering_status\"/>\n    <remap from=\"~/input/actuation_status\" to=\"/vehicle/status/actuation_status\"/>\n    <remap from=\"~/output/actuation_cmd\" to=\"/control/command/actuation_cmd\"/>\n    <remap from=\"~/output/steering_status\" to=\"/vehicle/status/steering_status\"/>\n  </node>"
            }
          ]
        },
        {
          title: "Create the simulator adapter package",
          detail: "Install interface.param.yaml and interface.launch.py through setuptools, with a console entry point named interface_node.",
          code: [
            {
              path: "autoware_carmaker_interface/setup.py",
              language: "PYTHON",
              startLine: 6,
              content: "setup(\n    name=package_name,\n    version=\"0.1.0\",\n    packages=find_packages(exclude=[\"test\"]),\n    data_files=[\n        (\"share/ament_index/resource_index/packages\", [\"resource/\" + package_name]),\n        (\"share/\" + package_name, [\"package.xml\"]),\n        (\"share/\" + package_name + \"/config\", glob(\"config/*.yaml\")),\n        (\"share/\" + package_name + \"/launch\", glob(\"launch/*launch.py\")),\n    ],\n    install_requires=[\"setuptools\"],\n    zip_safe=True,\n    maintainer=\"FSAI\",\n    maintainer_email=\"no@support.com\",\n    description=\"CarMaker adapter for Autoware\",\n    license=\"Apache-2.0\",\n    entry_points={\n        \"console_scripts\": [\n            \"interface_node = autoware_carmaker_interface.interface_node:main\",\n        ],\n    },\n)"
            }
          ]
        },
        {
          title: "Create the top-level Autoware wrapper",
          detail: "Include autoware.launch.xml with vehicle_model=carmaker_fsai_vehicle and sensor_model=carmaker_sensor_kit. Disable native sensing/localization and vehicle interface; enable system, LiDAR perception, planning, control and AD API.",
          code: [
            {
              path: "autoware_carmaker_launch/launch/carmaker.launch.py",
              language: "PYTHON",
              startLine: 32,
              content: "        launch_arguments={\n            \"map_path\": map_path,\n            \"vehicle_model\": \"carmaker_fsai_vehicle\",\n            \"sensor_model\": \"carmaker_sensor_kit\",\n            \"use_sim_time\": use_sim_time,\n            \"launch_vehicle\": \"true\",\n            \"launch_vehicle_interface\": \"false\",\n            \"launch_system\": \"true\",\n            \"system_run_mode\": \"planning_simulation\",\n            \"launch_system_monitor\": \"false\",\n            \"launch_dummy_diag_publisher\": \"true\",\n            \"diagnostic_graph_aggregator_graph_path\": diagnostic_graph_path,\n            \"launch_sensing\": \"false\",\n            \"launch_sensing_driver\": \"false\",\n            \"launch_localization\": \"false\",\n            \"launch_perception\": perception,\n            \"launch_planning\": \"true\",\n            \"launch_control\": \"true\",\n            \"launch_api\": \"true\",\n            \"is_simulation\": \"true\",\n            \"enable_all_modules_auto_mode\": \"true\",\n            \"perception_mode\": \"lidar\","
            }
          ]
        }
      ],
      files: [
        "carmaker_fsai_vehicle_description/config/vehicle_info.param.yaml",
        "carmaker_fsai_vehicle_description/urdf/vehicle.xacro",
        "carmaker_fsai_vehicle_launch/data/accel_map.csv",
        "carmaker_fsai_vehicle_launch/data/brake_map.csv",
        "carmaker_fsai_vehicle_launch/config/raw_vehicle_cmd_converter.param.yaml",
        "carmaker_fsai_vehicle_launch/launch/vehicle_interface.launch.xml",
        "carmaker_sensor_kit_description/config/sensor_kit_calibration.yaml",
        "carmaker_sensor_kit_description/urdf/sensors.xacro",
        "autoware_carmaker_interface/config/interface.param.yaml",
        "autoware_carmaker_launch/launch/carmaker.launch.py"
      ],
      config: [
        { key: "vehicle model", value: "carmaker_fsai_vehicle" },
        { key: "sensor model", value: "carmaker_sensor_kit" },
        { key: "LiDAR transform", value: "base_link → carmaker_lidar: 1.7, 0, 0.57 m" },
        { key: "IMU transform", value: "base_link → carmaker_imu: 1.3, 0, 0.3 m" },
        { key: "control conversion", value: "control_cmd → actuation_cmd" },
        { key: "steering limit", value: "±0.70 rad" },
        { key: "Autoware stack", value: "system + perception + planning + control + API" },
        { key: "external inputs", value: "launch_sensing=false; launch_localization=false" }
      ],
      commands: [
        "cd /home/klee/autoware.testvehicle\nsource /opt/ros/humble/setup.bash\nPATH=/usr/bin:/bin:$PATH /usr/bin/colcon build --symlink-install --packages-select carmaker_fsai_vehicle_description carmaker_fsai_vehicle_launch carmaker_sensor_kit_description carmaker_sensor_kit_launch autoware_carmaker_interface",
        "source install/setup.bash\nros2 pkg prefix carmaker_fsai_vehicle_description"
      ],
      verify: [
        "vehicle_info parameters appear on launched Autoware nodes",
        "TF contains base_link → carmaker_lidar and base_link → carmaker_imu",
        "raw_vehicle_cmd_converter publishes /control/command/actuation_cmd",
        "No package resolves from the main Autoware checkout"
      ],
      critical: true,
      bugs: [],
      result: "One node owns the simulator boundary, safety watchdog, diagnostics, and restart behavior."
    },
    {
      id: "traffic-light",
      number: "04",
      phase: "PERCEPTION",
      title: "Remove an unsupported camera path",
      summary: "Stop a nonessential perception component from taking down its container.",
      action: "Match enabled perception modules to the sensors the Sprint kit actually provides.",
      implementation: [
        {
          title: "Inventory the actual sensor set",
          detail: "Read the duplicated CarMaker vehicle infofile: inertial sensor B00, Lidar_F, front RGB/depth cameras, steering angle, and odometry are available; Autoware camera6/camera7 traffic-light contracts are not."
        },
        {
          title: "Constrain the Autoware launch",
          detail: "Keep launch_perception enabled in lidar mode, but pass use_traffic_light_recognition=false and traffic_light_recognition/use_high_accuracy_detection=false.",
          code: [
            {
              path: "autoware_carmaker_launch/launch/carmaker.launch.py",
              language: "PYTHON",
              startLine: 47,
              content: "            \"launch_perception\": perception,\n            \"launch_planning\": \"true\",\n            \"launch_control\": \"true\",\n            \"launch_api\": \"true\",\n            \"is_simulation\": \"true\",\n            \"enable_all_modules_auto_mode\": \"true\",\n            \"perception_mode\": \"lidar\",\n            # The pointcloud-based occupancy node requires exact timestamp\n            # equality between raw and segmented clouds. CarMaker's 10 ms\n            # ROS job can repeat its 20 ms LiDAR scan, so use Autoware's\n            # simulator configuration and consume the segmented cloud only.\n            \"occupancy_grid_map_method\": \"laserscan_based\",\n            \"input_obstacle_pointcloud\": \"true\",\n            \"input_obstacle_and_raw_pointcloud\": \"false\",\n            \"use_traffic_light_recognition\": \"false\",\n            \"traffic_light_recognition/use_high_accuracy_detection\": \"false\","
            }
          ]
        },
        {
          title: "Keep the sensor-kit diagnostics valid",
          detail: "Install the sensor-kit dummy and diagnostic aggregator metadata so Autoware recognizes the model without fabricating traffic-light camera streams."
        }
      ],
      files: [
        "autoware_carmaker_launch/launch/carmaker.launch.py",
        "carmaker_sensor_kit_launch/config/dummy_diag_publisher/sensor_kit.param.yaml",
        "Data/Vehicle/FS-AI_IMechE_ADS-DV_2026"
      ],
      config: [
        { key: "perception_mode", value: "lidar" },
        { key: "use_traffic_light_recognition", value: "false" },
        { key: "high_accuracy_detection", value: "false" }
      ],
      verify: [
        "No camera6/camera7 traffic-light container is launched",
        "LiDAR detection and obstacle segmentation containers remain active"
      ],
      critical: false,
      bugs: [
        {
          symptom: "traffic_light_node_container / camera7 exited with code -6.",
          diagnosis: "Default camera6/camera7 traffic-light pipelines started without supported Sprint traffic-light camera inputs.",
          fix: "Disabled traffic-light recognition and high-accuracy detection while retaining LiDAR perception.",
          result: "The unused composable camera pipeline no longer aborts."
        }
      ]
    },
    {
      id: "pointcloud",
      number: "05",
      phase: "DATA FORMAT",
      title: "Upgrade LiDAR to PointXYZIRC",
      summary: "Make every byte of the point cloud match Autoware's expected layout.",
      action: "Publish a packed 16-byte PointCloud2 point: XYZ float32 + intensity uint8 + return type uint8 + channel uint16.",
      implementation: [
        {
          title: "Read LidarRSI scan data",
          detail: "For every ScanPoint, use BeamID to recover azimuth/elevation from the beam table and halve LengthOF because the reported flight distance is out-and-back."
        },
        {
          title: "Convert polar returns to Cartesian XYZ",
          detail: "Compute x, y, z in the carmaker_lidar frame using ray length, elevation, and azimuth."
        },
        {
          title: "Normalize intensity safely",
          detail: "Replace non-finite intensity with zero, scale normalized 0–1 values to 0–255, then clamp before writing uint8."
        },
        {
          title: "Pack the Autoware point type",
          detail: "Create PointCloud2 fields x@0, y@4, z@8 as FLOAT32, intensity@12 and return_type@13 as UINT8, channel@14 as UINT16; set point_step=16.",
          code: [
            {
              path: "hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp",
              language: "C++",
              startLine: 298,
              content: "    msg.fields.resize(6);\n    const char* field_names[] = {\n        \"x\", \"y\", \"z\", \"intensity\", \"return_type\", \"channel\"};\n    const uint32_t field_offsets[] = {0, 4, 8, 12, 13, 14};\n    const uint8_t field_types[] = {\n        sensor_msgs::msg::PointField::FLOAT32,\n        sensor_msgs::msg::PointField::FLOAT32,\n        sensor_msgs::msg::PointField::FLOAT32,\n        sensor_msgs::msg::PointField::UINT8,\n        sensor_msgs::msg::PointField::UINT8,\n        sensor_msgs::msg::PointField::UINT16};\n    for (uint32_t field = 0; field < 6; ++field) {\n      msg.fields[field].name = field_names[field];\n      msg.fields[field].offset = field_offsets[field];\n      msg.fields[field].datatype = field_types[field];\n      msg.fields[field].count = 1;\n    }\n    msg.is_bigendian = false;\n    msg.point_step = 16;"
            }
          ]
        },
        {
          title: "Publish at the native scan stamp",
          detail: "Use LidarRSI[0].ScanTime for the raw source stamp and Lidar_F/carmaker_lidar as the frame contract. The adapter later pairs it to localization."
        }
      ],
      files: [
        "ros/ros2_ws/src/hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp",
        "ros/ros2_ws/src/hellocm_cmnode/src/CMNode_ROS2_HelloCM.hpp"
      ],
      config: [
        { key: "topic", value: "/carmaker/lidar/front/points" },
        { key: "frame", value: "carmaker_lidar" },
        { key: "point_step", value: "16 bytes" },
        { key: "CarMaker cluster cycle", value: "20 ms" },
        { key: "range", value: "0.3–300 m" },
        { key: "beam table", value: "LidarRSI_FS_autonomous" }
      ],
      verify: [
        "ros2 topic echo shows all six fields and point_step 16",
        "crop_box_filter no longer reports legacy PointXYZI",
        "The cloud frame matches the sensor-kit Xacro"
      ],
      critical: true,
      bugs: [
        {
          symptom: "“The pointcloud layout is compatible with PointXYZI. You may be using legacy code/data.”",
          diagnosis: "The old PointCloud/PointXYZI example did not satisfy the active Autoware PointXYZIRC contract.",
          fix: "Rebuilt the CMNode publisher with exact PointXYZIRC fields, offsets, types, frame, and point step.",
          result: "Autoware perception accepts the CarMaker LiDAR stream."
        }
      ]
    },
    {
      id: "readiness",
      number: "06",
      phase: "SYSTEM GATES",
      title: "Satisfy autonomous-mode readiness",
      summary: "Trace MRM emergency events back to missing upstream state.",
      action: "Publish transient initial pose and localization state, vehicle reports, and a simulator-specific diagnostic graph.",
      implementation: [
        {
          title: "Use CarMaker as the pose authority",
          detail: "Do not launch Autoware localization. Transform source odometry into map/base_link, apply optional map offsets, covariance, and publish /localization/kinematic_state plus map → base_link TF."
        },
        {
          title: "Complete the initialization contract",
          detail: "On the first valid odometry sample, publish INITIALIZED on /localization/initialization_state and the same pose on transient-local /initialpose3d.",
          code: [
            {
              path: "autoware_carmaker_interface/interface_node.py",
              language: "PYTHON",
              startLine: 391,
              content: "if not self.localization_initialized:\n    state = LocalizationInitializationState()\n    state.stamp = output.header.stamp\n    state.state = LocalizationInitializationState.INITIALIZED\n    self.localization_state_pub.publish(state)\n    self.initial_pose_pub.publish(pose)\n    self.localization_initialized = True"
            }
          ]
        },
        {
          title: "Publish vehicle reports every odometry cycle",
          detail: "Generate velocity, steering, gear, AUTONOMOUS control mode, disabled indicators/hazards, and actuation status with matching stamps."
        },
        {
          title: "Map gear commands to CarMaker selectors",
          detail: "Translate Autoware DRIVE, REVERSE, PARK, LOW and unknown/neutral commands to selectors 1, -1, -9, drive, and 0 respectively."
        },
        {
          title: "Publish safe diagnostics",
          detail: "Track wall-clock odometry/cloud freshness and expose counts, reset count, route-clear state, command hold, and sensor gating counters."
        },
        {
          title: "Tune only simulator-specific diagnostic timing",
          detail: "Use the custom graph to tolerate brief startup/perception stalls while leaving missing route, invalid/stale trajectory, and localization failures as autonomous-mode blockers."
        }
      ],
      files: [
        "autoware_carmaker_interface/autoware_carmaker_interface/interface_node.py",
        "autoware_carmaker_interface/config/interface.param.yaml",
        "autoware_carmaker_launch/config/autoware-carmaker.yaml"
      ],
      config: [
        { key: "pose frames", value: "map → base_link" },
        { key: "pose covariance x/y", value: "0.0001" },
        { key: "pose covariance yaw", value: "0.0001" },
        { key: "selector D/R/N/P", value: "1 / -1 / 0 / -9" },
        { key: "sensor stale timeout", value: "0.5 s" },
        { key: "initial pose QoS", value: "reliable + transient local" }
      ],
      verify: [
        "Autoware state changes Initializing → WaitingForRoute",
        "/initialpose3d can be read by a late subscriber",
        "Velocity, steering, gear and control-mode reports update continuously",
        "Autonomous remains blocked until route and trajectory are valid"
      ],
      critical: true,
      bugs: [
        {
          symptom: "MRM emergency toggled; localization/routing/trajectory were stale; actuation command timed out.",
          diagnosis: "No initialized localization meant no valid route; no route meant no trajectory; no trajectory meant no fresh control command. The brake watchdog was working correctly.",
          fix: "Completed localization and vehicle-status contracts, then required a valid goal and live trajectory before engagement.",
          result: "Autoware reaches WaitingForRoute and autonomous availability becomes healthy after planning is ready."
        }
      ]
    },
    {
      id: "sensor-gate",
      number: "07",
      phase: "TIME COHERENCE",
      title: "Gate sensors behind localization",
      summary: "Never let a cloud enter perception before its transform exists.",
      action: "Hold LiDAR and IMU until odometry establishes the current localization epoch.",
      implementation: [
        {
          title: "Track the latest localization stamp",
          detail: "After processing odometry, store the exact output stamp used for kinematic state and TF."
        },
        {
          title: "Retimestamp coupled sensors",
          detail: "Copy the latest localization stamp to LiDAR and IMU outputs so the sensor data and map → base_link transform are queryable in the same TF epoch."
        },
        {
          title: "Drop early data",
          detail: "If no localization stamp exists, increment waiting counters and return without publishing the cloud or IMU.",
          code: [
            {
              path: "autoware_carmaker_interface/interface_node.py",
              language: "PYTHON",
              startLine: 462,
              content: "        self.last_source_pointcloud_stamp_ns = source_stamp_ns\n        if self.latest_localization_stamp is None:\n            self.pointcloud_waiting_for_localization_count += 1\n            return\n\n        self.last_pointcloud_wall = time.monotonic()\n        self.pointcloud_count += 1\n        output = copy.deepcopy(msg)\n        output.header.stamp = copy.deepcopy(self.latest_localization_stamp)\n        output.header.frame_id = self.lidar_frame\n        self.pointcloud_pub.publish(output)"
            }
          ]
        },
        {
          title: "Use system-clock safety timers",
          detail: "Run the watchdog at 10 Hz and diagnostics at 1 Hz on ClockType.SYSTEM_TIME so a paused /clock cannot disable safety reporting."
        }
      ],
      files: [
        "autoware_carmaker_interface/autoware_carmaker_interface/interface_node.py"
      ],
      config: [
        { key: "watchdog timer", value: "0.1 s system clock" },
        { key: "diagnostic timer", value: "1.0 s system clock" },
        { key: "sensor pairing", value: "latest odometry localization stamp" }
      ],
      verify: [
        "No forwarded cloud has an empty frame",
        "Accepted cloud stamps have a matching map → base_link transform",
        "Waiting counters increase instead of TF lookup failures"
      ],
      critical: true,
      bugs: [
        {
          symptom: "Occupancy TF lookup received an empty source frame.",
          diagnosis: "A first sensor sample could reach perception before valid odometry-derived localization and TF.",
          fix: "Added localization-aware sensor gating and epoch counters.",
          result: "Perception starts only with a coherent frame and timestamp."
        }
      ]
    },
    {
      id: "restart",
      number: "08",
      phase: "LIFECYCLE",
      title: "Survive repeated TestRuns",
      summary: "Keep Autoware and RViz alive when CarMaker time returns to zero.",
      action: "Make /clock monotonic and treat raw odometry regression as a new simulation epoch.",
      implementation: [
        {
          title: "Make CMROSIF clock monotonic",
          detail: "Track raw SimCore time, published ROS time, and a cumulative offset. If the next published value would not advance, increase the offset by at least one clock cycle.",
          code: [
            {
              path: "cmrosutils/src/cmrosif.cpp",
              language: "C++",
              startLine: 97,
              content: "    job->registerCallback([this](rosgraph_msgs::msg::Clock& msg) {\n      const auto raw_sim_time_ns =\n          static_cast<int64_t>(1e6 * std::round(1e3 * SimCore.Time));\n      auto published_time_ns = clock_offset_ns_ + raw_sim_time_ns;\n\n      if (last_published_clock_ns_ >= 0 &&\n          published_time_ns <= last_published_clock_ns_) {\n        const auto minimum_step_ns = std::max<int64_t>(\n            1, static_cast<int64_t>(clock_cycle_time_) * 1000000);\n        clock_offset_ns_ +=\n            last_published_clock_ns_ + minimum_step_ns - published_time_ns;\n        published_time_ns = clock_offset_ns_ + raw_sim_time_ns;"
            }
          ]
        },
        {
          title: "Detect a new TestRun independently",
          detail: "Keep raw odometry stamps in the adapter. A regression larger than 0.05 s identifies a new TestRun even though published /clock stays monotonic."
        },
        {
          title: "Enter command hold",
          detail: "Reset command state, steering, selector, localization stamp, and sensor freshness; publish INITIALIZING and command zero gas plus 0.35 brake.",
          code: [
            {
              path: "autoware_carmaker_interface/interface_node.py",
              language: "PYTHON",
              startLine: 582,
              content: "        self.simulation_reset_count += 1\n        self.localization_initialized = False\n        self.command_active = False\n        self.last_command_wall = None\n        self.safe_stop_sent = False\n        self.last_pointcloud_wall = None\n        self.latest_localization_stamp = None\n        self.latest_steering_tire_angle = 0.0\n        self.current_selector = self.selector_drive\n        self.current_gear_report = GearReport.DRIVE\n        self.command_hold_active = True\n        self.safe_stop_sent = True\n        self.route_clear_pending = self.clear_route_on_simulation_reset\n        if not self.route_clear_pending:\n            self.command_hold_active = False\n        self.next_route_clear_attempt_wall = 0.0\n\n        state = LocalizationInitializationState()\n        state.stamp = self.get_clock().now().to_msg()\n        state.state = LocalizationInitializationState.INITIALIZING\n        self.localization_state_pub.publish(state)\n        self._publish_control(0.0, self.timeout_brake, 0.0, 0.0)"
            }
          ]
        },
        {
          title: "Clear the previous mission",
          detail: "Call /api/routing/clear_route asynchronously. If unavailable or rejected, keep safe brake and retry once per second."
        },
        {
          title: "Initialize the new epoch",
          detail: "First new odometry republishes initialized state, /initialpose3d and TF. Release command hold only after route clear succeeds."
        }
      ],
      files: [
        "ros/ros2_ws/src/cmrosutils/src/cmrosif.cpp",
        "ros/ros2_ws/src/cmrosutils/include/cmrosutils/cmrosif.hpp",
        "autoware_carmaker_interface/autoware_carmaker_interface/interface_node.py"
      ],
      config: [
        { key: "reset tolerance", value: "0.05 s" },
        { key: "route-clear retry", value: "1.0 s" },
        { key: "safe brake", value: "0.35" },
        { key: "clear route on reset", value: "true" }
      ],
      verify: [
        "/clock never moves backward across two TestRuns",
        "Reset log shows previous and new raw times",
        "Old route is cleared and Autoware returns to WaitingForRoute",
        "Autoware and RViz remain running"
      ],
      critical: true,
      bugs: [
        {
          symptom: "After a TestRun restart, the map/vehicle view disappeared and old route state remained.",
          diagnosis: "SimCore.Time jumped backward, invalidating ROS timers, TF caches, RViz filters, and the previous route.",
          fix: "Added a CMROSIF clock offset, reset localization, applied safe brake, cleared the route with retry, and republished initial pose.",
          result: "A new run returns cleanly to WaitingForRoute without restarting Autoware."
        }
      ]
    },
    {
      id: "first-scan",
      number: "09",
      phase: "TIME COHERENCE",
      title: "Pair the first scan with the new epoch",
      summary: "Handle sensors that reset before the odometry callback observes the restart.",
      action: "Track raw odometry, LiDAR, and IMU timestamps independently.",
      implementation: [
        {
          title: "Keep three raw-time histories",
          detail: "Store last_source_odometry_stamp_ns, last_source_pointcloud_stamp_ns, and last_source_imu_stamp_ns before replacing message headers with monotonic ROS time."
        },
        {
          title: "Detect sensor-first restart order",
          detail: "If a sensor regresses while odometry still belongs to the previous epoch, clear latest_localization_stamp immediately.",
          code: [
            {
              path: "autoware_carmaker_interface/interface_node.py",
              language: "PYTHON",
              startLine: 450,
              content: "        source_stamp_ns = stamp_to_nanoseconds(msg.header.stamp)\n        if sensor_precedes_odometry_epoch(\n            self.last_source_pointcloud_stamp_ns,\n            source_stamp_ns,\n            self.last_source_odometry_stamp_ns,\n            self.simulation_reset_tolerance_sec,\n        ):\n            # CarMaker sensor jobs can run before the first odometry job of a\n            # restarted TestRun. Do not stamp the new scan with the previous\n            # TestRun's pose.\n            self.latest_localization_stamp = None"
            }
          ]
        },
        {
          title: "Wait for the odometry reset callback",
          detail: "Drop new-epoch sensor messages until odometry detects the TestRun reset and creates the new localization stamp."
        },
        {
          title: "Test the boundary helper",
          detail: "Unit-test the sequence 58 s sensor → 0.010 s sensor while odometry remains at 58 s, then verify it clears once odometry reaches 0.012 s."
        }
      ],
      files: [
        "autoware_carmaker_interface/autoware_carmaker_interface/interface_node.py",
        "autoware_carmaker_interface/test/test_interface_helpers.py"
      ],
      config: [
        { key: "first LiDAR sample", value: "0.010 s" },
        { key: "first odometry/TF sample", value: "0.012 s" },
        { key: "regression helper", value: "sensor_precedes_odometry_epoch()" }
      ],
      verify: [
        "The helper test detects only the sensor-before-odometry reset case",
        "RViz no longer drops the first accepted restart cloud",
        "CenterPoint no longer requests TF before the cache begins"
      ],
      critical: true,
      bugs: [
        {
          symptom: "LiDAR requested TF at 0.010 s, but the earliest transform was 0.012 s.",
          diagnosis: "The new-epoch cloud arrived before the first new-epoch odometry transform.",
          fix: "Detect sensor-before-odometry regression, invalidate old localization, and drop data until the matching epoch exists.",
          result: "The first accepted cloud is inside the TF cache."
        }
      ]
    },
    {
      id: "occupancy",
      number: "10",
      phase: "PLANNING INPUT",
      title: "Unblock the occupancy grid",
      summary: "A valid route still cannot move the car without behavior-planning input.",
      action: "Use the simulator-style segmented-cloud occupancy implementation.",
      implementation: [
        {
          title: "Trace the planner dependency backward",
          detail: "Start at missing trajectory_follower/control_cmd, confirm /planning/trajectory is absent, then read behavior_path_planner's waiting for occupancy_grid message."
        },
        {
          title: "Compare producer timing",
          detail: "CMNode jobs execute every 10 ms, but SensorCluster.0 LidarRSI updates every 20 ms. Repeated raw scans and delayed segmentation create different latest stamps."
        },
        {
          title: "Select the simulator occupancy mode",
          detail: "Set occupancy_grid_map_method=laserscan_based, input_obstacle_pointcloud=true, and input_obstacle_and_raw_pointcloud=false.",
          code: [
            {
              path: "autoware_carmaker_launch/launch/carmaker.launch.py",
              language: "PYTHON",
              startLine: 54,
              content: "            # The pointcloud-based occupancy node requires exact timestamp\n            # equality between raw and segmented clouds. CarMaker's 10 ms\n            # ROS job can repeat its 20 ms LiDAR scan, so use Autoware's\n            # simulator configuration and consume the segmented cloud only.\n            \"occupancy_grid_map_method\": \"laserscan_based\",\n            \"input_obstacle_pointcloud\": \"true\",\n            \"input_obstacle_and_raw_pointcloud\": \"false\","
            }
          ]
        },
        {
          title: "Preserve the rest of LiDAR perception",
          detail: "Keep crop-box filtering, obstacle segmentation, detection and map comparison active; change only the occupancy input synchronization contract."
        }
      ],
      files: [
        "autoware_carmaker_launch/launch/carmaker.launch.py",
        "Data/Vehicle/FS-AI_IMechE_ADS-DV_2026"
      ],
      config: [
        { key: "ROS job", value: "10 ms" },
        { key: "LidarRSI scan", value: "20 ms" },
        { key: "occupancy method", value: "laserscan_based" },
        { key: "occupancy input", value: "segmented obstacle cloud only" }
      ],
      verify: [
        "LaserscanBasedOccupancyGridMapNode is loaded",
        "/perception/occupancy_grid_map/map publishes continuously",
        "behavior_path_planner stops waiting for occupancy_grid"
      ],
      critical: true,
      bugs: [
        {
          symptom: "Route accepted, but behavior planner waited for occupancy_grid and controller waited for trajectory.",
          diagnosis: "The pointcloud-pair node demanded exactly equal raw and segmented timestamps. A 10 ms ROS job repeated a 20 ms LiDAR scan, so the latest stamps never paired.",
          fix: "Set occupancy_grid_map_method=laserscan_based and consume only the segmented obstacle cloud.",
          result: "Occupancy publishes, behavior planning advances, and a trajectory reaches control."
        }
      ]
    },
    {
      id: "map",
      number: "11",
      phase: "MAP ALIGNMENT",
      title: "Put the lane under the vehicle",
      summary: "Align generated Lanelet geometry with the actual IPGRoad reference.",
      action: "Regenerate the Sprint map with zero lane-center offset.",
      implementation: [
        {
          title: "Read the CarMaker road source",
          detail: "Parse Link.0.Node0 and each IPGRoad segment. Sample straight/turn geometry and Hermite PointList spans at 1 m spacing."
        },
        {
          title: "Generate lane boundaries",
          detail: "Offset the sampled centerline by half of a 4 m lane width, chunk boundaries every 20 samples, and write one-way road lanelets with a 15 m/s speed limit."
        },
        {
          title: "Generate the point-cloud map",
          detail: "Read terrain OBJ vertices and write an ASCII PCD with x, y, z, intensity. Write map_projector_info.yaml with projector_type: Local."
        },
        {
          title: "Align the centerline",
          detail: "Use lane-center-offset=0.0 because the FSAI vehicle starts on the IPGRoad reference. The earlier 2.0 m offset was the pull-out failure.",
          code: [
            {
              path: "autoware_carmaker_launch/generate_sprint_map.py",
              language: "PYTHON",
              startLine: 223,
              content: "parser.add_argument(\n    \"--lane-center-offset\",\n    type=float,\n    default=0.0,\n    help=(\n        \"Lateral offset from the IPGRoad reference line; \"\n        \"the FSAI vehicle starts on 0.0\"\n    ),\n)"
            }
          ]
        },
        {
          title: "Install the generated map path",
          detail: "Point the wrapper's map_path to /home/klee/autoware.testvehicle/maps/fs_autonomous_sprint."
        }
      ],
      files: [
        "autoware_carmaker_launch/autoware_carmaker_launch/generate_sprint_map.py",
        "maps/fs_autonomous_sprint/lanelet2_map.osm",
        "maps/fs_autonomous_sprint/pointcloud_map.pcd",
        "maps/fs_autonomous_sprint/map_projector_info.yaml"
      ],
      config: [
        { key: "sampling", value: "1.0 m" },
        { key: "lane width", value: "4.0 m" },
        { key: "lane-center offset", value: "0.0 m" },
        { key: "speed limit", value: "15 m/s" },
        { key: "start pose", value: "256.102544, -120.416743" },
        { key: "start-center distance", value: "≈ 0.124 m" }
      ],
      commands: [
        "cd /home/klee/autoware.testvehicle\npython3 src/carmaker_integration/autoware_carmaker_launch/autoware_carmaker_launch/generate_sprint_map.py --road <Sprint.rd5> --terrain <terrain.obj> --output maps/fs_autonomous_sprint --spacing 1.0 --lane-width 4.0 --lane-center-offset 0.0"
      ],
      verify: [
        "Lanelet map and PCD load in RViz",
        "Initial pose lies on the lane center, not 2 m beside it",
        "Mission planner accepts a forward goal on connected lanelets"
      ],
      critical: true,
      bugs: [
        {
          symptom: "ClothoidPullOut repeatedly failed with relative lateral pose -2.000 m.",
          diagnosis: "The map generator placed the lane center 2 m away from the real CarMaker start line.",
          fix: "Changed the default lane-center offset from 2.0 to 0.0 and regenerated Lanelet2/PCD data.",
          result: "The start pose is approximately 0.124 m from the generated centerline."
        }
      ]
    },
    {
      id: "torque",
      number: "12",
      phase: "ACTUATION",
      title: "Turn positive acceleration into forward torque",
      summary: "Trace a valid command through the vehicle's physical interpretation.",
      action: "Align the CarMaker gas map with Autoware's zero-pedal convention.",
      implementation: [
        {
          title: "Trace command units through the chain",
          detail: "Inspect /control/command/control_cmd, converter output /control/command/actuation_cmd, /carmaker/VehicleControl.gas, and the CarMaker vehicle's GasInterpret model."
        },
        {
          title: "Calculate the old torque",
          detail: "With TrqFull=55 and TrqZero=-55, linear torque is 110×gas−55. A typical gas=0.125 therefore requests -41.25 Nm."
        },
        {
          title: "Match zero pedal to zero drive torque",
          detail: "In the duplicated vehicle only, retain Linear mode and TrqFull=55 but set TrqZero=0.",
          code: [
            {
              path: "Data/Vehicle/FS-AI_IMechE_ADS-DV_2026",
              language: "CARMAKER INFOFILE",
              startLine: 379,
              content: "PowerTrain.Control.GasInterpret.Mode = Linear\nPowerTrain.Control.GasInterpret.TrqFull = 55\nPowerTrain.Control.GasInterpret.TrqZero = 0"
            }
          ]
        },
        {
          title: "Confirm steering semantics separately",
          detail: "Review the vehicle model documentation: VehicleControl.Steering.Ang is road-wheel angle, so steering command and feedback ratios remain 1.0 rather than introducing a steering-wheel ratio."
        }
      ],
      files: [
        "Data/Vehicle/FS-AI_IMechE_ADS-DV_2026",
        "carmaker_fsai_vehicle_launch/data/accel_map.csv",
        "autoware_carmaker_interface/config/interface.param.yaml"
      ],
      config: [
        { key: "GasInterpret.Mode", value: "Linear" },
        { key: "TrqFull", value: "55 Nm" },
        { key: "TrqZero", value: "0 Nm" },
        { key: "gas=0.125", value: "+6.875 Nm before final drive" },
        { key: "steering ratio", value: "1.0 road-wheel angle" },
        { key: "steering signs", value: "+1 command / +1 feedback" }
      ],
      verify: [
        "Positive actuation_cmd produces positive VehicleControl.gas",
        "Vehicle longitudinal velocity becomes positive",
        "Brake is zero during positive acceleration",
        "Low-speed steering direction matches the planned curve"
      ],
      critical: true,
      bugs: [
        {
          symptom: "A moving trajectory and positive acceleration command existed, but the car remained at the start.",
          diagnosis: "TrqZero=-55 made every gas command below 50% request negative torque; Autoware normally commanded roughly 0.04–0.125.",
          fix: "Set TrqZero=0 only in the duplicated FSAI vehicle.",
          result: "gas=0.125 changes from about -41.25 Nm to +6.875 Nm before final drive."
        }
      ]
    },
    {
      id: "validation",
      number: "13",
      phase: "VERIFICATION",
      title: "Prove the complete contract",
      summary: "Separate real runtime blockers from test-harness and shutdown-only noise.",
      action: "Build both sides, run helper tests, exercise a synthetic full ROS contract, and test route-reset retry.",
      implementation: [
        {
          title: "Build all seven integration packages",
          detail: "Source ROS Humble, force system Python, and use colcon --symlink-install for message, interface, vehicle, sensor, and wrapper packages."
        },
        {
          title: "Build the duplicated CM15 executable",
          detail: "Run build.sh 2, which cleans the CarMaker src target then calls make cm-ros2 to compile CMROSIF and the modified CMNode shared library."
        },
        {
          title: "Start in dependency order",
          detail: "Open the duplicated project with CMStart.sh 2; start isolated Autoware; load FS_autonomous_Sprint_LTS; start hellocm/CMROSIF; then start the TestRun."
        },
        {
          title: "Verify inputs before choosing a route",
          detail: "Check /clock, CarMaker odometry/LiDAR, localization state, initialpose3d and occupancy. Wait for WaitingForRoute."
        },
        {
          title: "Create the route and engage",
          detail: "Use 2D Goal Pose on a forward lane center, confirm routing state, mission route, positive-velocity trajectory and VehicleControl direction, then request autonomous mode."
        },
        {
          title: "Exercise restart safety",
          detail: "End and restart the TestRun without stopping Autoware. Confirm reset detection, brake hold, route clear, republished pose, and WaitingForRoute."
        },
        {
          title: "Run automated contracts",
          detail: "Run six helper tests plus synthetic sensor→perception→planning→control validation and a route-clear rejection/retry scenario."
        }
      ],
      files: [
        "autoware_carmaker_launch/README.md",
        "autoware_carmaker_interface/test/test_interface_helpers.py",
        "FSAI_LTS_2026_CM15_Autoware/build.sh",
        "FSAI_LTS_2026_CM15_Autoware/CMStart.sh"
      ],
      config: [
        { key: "connected run time", value: "use_sim_time=true" },
        { key: "synthetic harness", value: "use_sim_time=false unless it publishes /clock" },
        { key: "supported TestRun", value: "FS_autonomous_Sprint_LTS" },
        { key: "successful max trajectory", value: "≈ 4.167 m/s" },
        { key: "successful max accel command", value: "≈ 0.125" },
        { key: "helper tests", value: "6 passed" }
      ],
      commands: [
        "cd /home/klee/autoware.testvehicle\nsource /opt/ros/humble/setup.bash\nPATH=/usr/bin:/bin:$PATH /usr/bin/colcon build --symlink-install --packages-select vehiclecontrol_msgs autoware_carmaker_interface carmaker_fsai_vehicle_description carmaker_fsai_vehicle_launch carmaker_sensor_kit_description carmaker_sensor_kit_launch autoware_carmaker_launch",
        "cd /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware\nPATH=/usr/bin:/bin:$PATH ./build.sh 2\n./CMStart.sh 2",
        "cd /home/klee/autoware.testvehicle\nsource /opt/ros/humble/setup.bash\nsource install/setup.bash\nros2 launch autoware_carmaker_launch carmaker.launch.py",
        "cd /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware\nsource ros_setup.bash 2\nros2 launch hellocm hellocm.launch.py use_sim_time:=True rviz2:=False"
      ],
      verify: [
        "Autoware reports WaitingForRoute before goal selection",
        "Occupancy, route, trajectory and control topics are live in order",
        "Autonomous mode becomes available and control is enabled",
        "The CarMaker vehicle leaves the starting point",
        "A second TestRun works without restarting Autoware"
      ],
      critical: true,
      bugs: [
        {
          symptom: "Synthetic validation froze and colcon initially reported zero helper tests.",
          diagnosis: "The harness enabled simulated time without publishing /clock; Python test metadata also did not discover the helper file.",
          fix: "Use wall time only for the synthetic harness and invoke the six helper tests directly with pytest.",
          result: "Moving trajectory, positive actuation, autonomous availability, reset safety, and six helpers all passed."
        },
        {
          symptom: "motion_planning_container sometimes exited -11 after Ctrl-C.",
          diagnosis: "The trace was confined to MotionVelocityPlanner teardown after SIGINT, not the live driving path.",
          fix: "Classified it as a shutdown-only upstream issue; keep Autoware alive between TestRuns.",
          result: "The teardown fault is documented but does not block the working closed loop."
        }
      ]
    }
  ],

  restart: [
    { state: "RUNNING", detail: "Route + trajectory + fresh actuation" },
    { state: "TIME REGRESSION", detail: "Raw stamp falls from 58.290 s to 0.010 s", alert: true },
    { state: "SAFE HOLD", detail: "Zero gas · 35% brake · reject old-epoch sensors" },
    { state: "REINITIALIZE", detail: "Advance /clock epoch · publish pose + TF" },
    { state: "CLEAR ROUTE", detail: "Call AD API; retry until confirmed" },
    { state: "WAITING FOR ROUTE", detail: "Operator selects a new goal", success: true }
  ]
};
