# Demonstration and reproduction record

## Final course demonstration

| Item | Record |
| --- | --- |
| Setting | Autonomous System Platform course, May–June 2026 |
| Platform | PX4 SITL / Gazebo simulation with ROS 2 |
| Mission package | `uav_waypoint_mission` |
| Outcome | Sequential waypoint flight and final landing completed |
| Source version | Archive confirmed by the project author as the final version |
| Confirmation | The project author reconfirmed that the final demonstration worked correctly |

The reported result is an integrated simulation demonstration. The included CSV contains nine targets; the exact final command-line overrides and landing handler have not yet been recovered. Route completion and landing are recorded as the demonstration outcome without attributing the entire flight stack to one node.

## Repository checks

The code collection was inspected offline on Windows. File integrity, Python syntax, package XML, route contents and documentation links are recorded in [VALIDATION.md](VALIDATION.md). These checks do not rerun the demonstration.

## Reproduction record to add

A future recorded replay should capture the exact launch commands, map/PX4 origin alignment, TF tree, landing-command receiver, waypoint status and simulator view. One continuous clip from mission start through final landing would make a useful visual overview. No numerical tracking-error or repeated-trial success-rate measurements have been supplied.
