# Source provenance

## Confirmed source

- Archive: `autosystem_platform_ws_full_20260618_133457.tar.gz`
- Archive size: 3,109,823,287 bytes.
- Archive SHA-256: `44143853d00d691389a5f2d77bb1ca1778955f670529f1a297831a45e2d4df9b`
- The project author confirmed this archive as the final demonstration version.
- Root inside the archive: `autosystem_platform_ws/utilities_pkg/`.

## Collection map

| Archive directory under `utilities_pkg/` | Repository directory | Files | Purpose |
| --- | --- | ---: | --- |
| `uav_waypoint_mission/` | `src/uav_waypoint_mission/` | 12 | Main waypoint implementation, launch, route and package metadata |
| `px4_ros_com/` | `src/px4_ros_com/` | 26 | Archived offboard controller, transform utilities and build files |
| `px4_msgs/` | `src/px4_msgs/` | 231 | Matching ROS message definitions |
| `gazebo_env_setup/` | `src/gazebo_env_setup/` | 12 | Gazebo bridges, TF, launch and visualization configuration |

All **281 imported files** retain their original bytes. The [manifest](SOURCE_MANIFEST.csv) records original paths, destination paths, sizes, archive modes and SHA-256 hashes. Collection adds only portfolio documentation and repository housekeeping; it does not alter flight behavior.

The wider archive contains other robot missions, tracking packages, PX4 firmware, simulation resources and generated builds. This repository focuses on the waypoint package and its ROS-side integration chain. The separate `robot_control/robot_control/uav/waypoint_mission.py` is another implementation and is not substituted for `uav_waypoint_mission`.

## Licensing and attribution

Existing source headers, metadata and license files are preserved. `px4_ros_com` and `px4_msgs` include their own [controller license](../src/px4_ros_com/LICENSE) and [message license](../src/px4_msgs/LICENSE). The mission package declares MIT in its metadata; this collection does not add a new blanket license or relicense the integration dependencies. Consult each package's existing notices when reusing code.
