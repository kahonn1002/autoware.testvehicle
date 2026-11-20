#!/usr/bin/env python3
"""Install the ISO additions in the existing duplicated CM15 project with backups."""

import argparse
import json
import re
import shutil
import subprocess
import tempfile
from pathlib import Path


def replace(text, key, value):
    pattern = r"^"+re.escape(key)+r"\s*=.*$"
    if re.search(pattern,text,re.M):
        return re.sub(pattern,key+" = "+str(value),text,flags=re.M)
    return text.rstrip()+"\n"+key+" = "+str(value)+"\n"


def prepare_scenario(original):
    text = replace(original,"Vehicle","Autoware/DemoCar_ISO")
    # Start before the gates for short commissioning runs. Benchmark original stays at zero.
    text = replace(text,"Vehicle.StartPos","450.00 0")
    start = text.index("DrivMan.nMan =")
    end = text.index("DrivMan.SpeedUnit =",start)
    hold = """DrivMan.nMan = 1
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
"""
    text = text[:start]+hold+text[end:]
    text = replace(text,"Driver.Vel.CruisingSpeed","10")
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project",type=Path,required=True)
    parser.add_argument("--dry-run",action="store_true")
    args = parser.parse_args()
    project = args.project.resolve(strict=True)
    if project.name != "FSAI_LTS_2026_CM15_Autoware" or not (project/"src/CM_Main.cpp").is_file():
        raise ValueError("Expected the duplicated FSAI CM15 Autoware project")
    here = Path(__file__).resolve().parent
    patch = here/"iso_ros.patch"
    lifecycle_patch = here/"iso_camera_lifecycle.patch"
    additional_patches = [lifecycle_patch, here/"iso_camera_epoch.patch", here/"iso_control_watchdog.patch",
                          here/"iso_shared_epoch.patch", here/"iso_watchdog_diagnostics.patch"]
    relative_sources = [
        "ros/ros2_ws/src/hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp",
        "ros/ros2_ws/src/carmaker_rsds_client/src/carmaker_rsds_client_node.cpp",
        "ros/ros2_ws/src/carmaker_rsds_client/launch/carmaker_rsds_client.launch.py",
    ]
    for relative in relative_sources:
        path = project/relative
        if path.is_symlink() or not path.is_file() or not path.resolve().is_relative_to(project):
            raise ValueError(f"Unexpected source target: {path}")
    # Prepare the entire patch chain in a temporary copy before changing any
    # project source. Unknown/partially applied changes fail closed here.
    stage = Path(tempfile.mkdtemp(prefix="iso-overlay-check-"))
    for relative in relative_sources:
        destination = stage/relative
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(project/relative,destination)
    for optional_patch in reversed(additional_patches):
        reverse_check = subprocess.run(["git","apply","--reverse","--check",str(optional_patch)],cwd=stage,capture_output=True,text=True)
        if reverse_check.returncode == 0:
            subprocess.run(["git","apply","--reverse",str(optional_patch)],cwd=stage,check=True)
    check = subprocess.run(["git","apply","--check",str(patch)],cwd=stage,capture_output=True,text=True)
    already_applied = False
    if check.returncode:
        reverse = subprocess.run(["git","apply","--reverse","--check",str(patch)],cwd=stage,capture_output=True,text=True)
        if reverse.returncode:
            raise RuntimeError("CarMaker sources do not match patch: "+check.stderr)
        already_applied = True
    else:
        subprocess.run(["git","apply",str(patch)],cwd=stage,check=True)
    for optional_patch in additional_patches:
        subprocess.run(["git","apply","--check",str(optional_patch)],cwd=stage,check=True)
        subprocess.run(["git","apply",str(optional_patch)],cwd=stage,check=True)
    targets = relative_sources+[
        "ros/ros2_ws/src/vehiclecontrol_msgs/msg/VehicleControl.msg",
        "Data/Vehicle/Autoware/DemoCar_ISO", "Data/TestRun/LaneChange_ISO_Autoware",
        "Data/Vehicle/Autoware/DemoCar_ISO_NoRSI", "Data/TestRun/LaneChange_ISO_Autoware_NoRSI"]
    for relative in targets:
        target = project/relative
        if not target.resolve().is_relative_to(project) or target.is_symlink():
            raise ValueError(f"Unexpected target {target}")
        if target.exists() and not target.is_file():
            raise ValueError(f"Unexpected target {target}")
    vehicle = (project/"Data/Vehicle/Examples/DemoCar").read_text()
    vehicle = replace(vehicle,"Autoware.SensorTFExternal",1)
    scenario = prepare_scenario((project/"Data/TestRun/LaneChange_ISO_Camera_RSI").read_text())
    no_rsi_vehicle = vehicle
    for index in range(int(re.search(r"^Sensor.N\s*=\s*(\d+)",vehicle,re.M).group(1))):
        param = re.search(r"^Sensor\."+str(index)+r"\.Ref.Param\s*=\s*(\d+)",vehicle,re.M).group(1)
        kind = re.search(r"^Sensor.Param\."+param+r"\.Type\s*=\s*(\w+)",vehicle,re.M).group(1)
        if kind in ("LidarRSI","CameraRSI"):
            no_rsi_vehicle = replace(no_rsi_vehicle,f"Sensor.{index}.Active",0)
    print(json.dumps({"project":str(project),"targets":targets,"patch_already_applied":already_applied},indent=2))
    if args.dry_run:
        return
    backup = Path(tempfile.mkdtemp(prefix="iso-overlay-",dir=project))
    for relative in targets:
        target = project/relative
        if target.exists():
            if target.is_symlink() or not target.is_file():
                raise ValueError(f"Unexpected target {target}")
            destination = backup/relative
            destination.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(target,destination)
    for relative in relative_sources:
        shutil.copy2(stage/relative,project/relative)
    msg_source = here.parent/"vehiclecontrol_msgs/msg/VehicleControl.msg"
    shutil.copy2(msg_source,project/targets[3])
    vehicle_path = project/targets[4]
    vehicle_path.parent.mkdir(parents=True,exist_ok=True)
    vehicle_path.write_text(vehicle)
    (project/targets[5]).write_text(scenario)
    (project/targets[6]).write_text(no_rsi_vehicle)
    (project/targets[7]).write_text(replace(scenario,"Vehicle","Autoware/DemoCar_ISO_NoRSI"))
    print(f"Installed ISO overlay; backups: {backup}")


if __name__ == "__main__":
    main()
