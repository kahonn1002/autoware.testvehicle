# CarMaker 15 integration for Autoware

This isolated package set connects the Formula Student CarMaker project at
`/home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware` to the isolated
Autoware workspace in `/home/klee/autoware.testvehicle`. It does not require
changes to `/home/klee/autoware` or to the original CarMaker project.

The first supported configuration uses the `FS_autonomous_Sprint_LTS` TestRun,
CarMaker ground-truth localization, IMU, steering feedback, and the CarMaker
LidarRSI sensor. Autoware's final actuation command is converted to gas, brake,
selector, and steering commands with a stale-command brake watchdog.

See `autoware_carmaker_launch/README.md` for the build, startup, engagement, and
validation procedure.

See `ISO_LANECHANGE.md` for the commissioning-only `iso_lanechange_demo`
profile (copied `LaneChange_ISO_Autoware` TestRun, synthetic map/PCD, current
test evidence, and explicitly unverified parts).

Other TestRuns are converted with `testrun_to_autoware` (map, scenario and launch
profile from CarMaker's own road evaluation); see section 5 of
`autoware_carmaker_launch/README.md`.
