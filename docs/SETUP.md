# Setup and integration

## Environment and scope

The course used ROS 2 Humble, PX4 SITL, Gazebo, QGroundControl and Micro XRCE-DDS. This collection includes four ROS packages from the final workspace. The PX4 firmware tree, simulator world/models, generated builds, and binary tools remain in the original archive.

The included `gazebo_env_setup` build requests `gz-msgs10` and `gz-transport13`. A generic Gazebo installation is not sufficient unless these dependencies and the course assets are available. PX4 and DDS versions must match the archived `px4_msgs` definitions; an exact firmware commit has not been established from this collection.

## Build

For the mission package alone, with ROS 2 and its dependencies installed:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src/uav_waypoint_mission
source install/setup.bash
```

For the four-package integration snapshot, resolve its C++/Gazebo/ROS dependencies in the course environment before running:

```bash
colcon build --base-paths src
source install/setup.bash
```

Both commands describe the intended ROS workflow; this Windows collection session did not run `colcon` or SITL. Source build declarations are preserved, including incomplete dependency declarations that may require additions in a clean workspace. For example, the controller uses `std_msgs` and `tf2_ros`, while its CMake target does not explicitly list both. See [validation](VALIDATION.md).

## Archived interfaces that need matching

### Coordinates

- The mission launch defaults to `command_pose_mode:=local_ned`. This subtracts the first TF position and converts ENU-like map coordinates into local NED values.
- The included [offboard controller](../src/px4_ros_com/src/examples/offboard/offboard_control.cpp) already converts pose positions and orientations from ENU to NED in `pose_callback`; it does not branch on the incoming frame ID.
- Using both conversions would convert the command twice. `map_enu` avoids that double axis conversion for this controller, but **map origin alignment with PX4 local position must also be checked**: the controller applies no map-origin offset.
- The exact command-line overrides used during the final demonstration were not recorded in this collection. Preserve the working course configuration when reproducing it.

### TF

The mission expects `map → x500_gimbal_0/base_link`, with fallback `x500_gimbal_0`. The controller's altitude guard instead uses the hard-coded frame `base_link`. Check the actual TF tree and ensure this frame identifies the same UAV before relying on that guard.

The [pose broadcaster](../src/gazebo_env_setup/src/pose_tf_broadcaster.cpp) subscribes directly to Gazebo Transport pose topics and publishes ROS TF. The separate `ros_gz_bridge` launch provides `/clock` and other simulation topics.

### Completion and landing

The waypoint node publishes `/command/land` (`std_msgs/msg/Bool`). The included offboard controller subscribes to `/command/pose`, `/command/twist`, `/command/disarm`, and `/gimbal_pitch_degree`; it has no `/command/land` subscription.

Consequently, publishing `land` alone does not establish a complete landing path with these two nodes. The final course system completed landing, as confirmed by the project author; the exact landing handler and launch combination remain to be identified for standalone reproduction. Begin an interface-only SITL check with `finish_action:=hold` and connect the course landing path before testing other finish modes.

`COMPLETE` is a mission-node state, not an acknowledgement that PX4 has touched down. The `land_then_disarm` mode checks altitude before issuing disarm, while the direct `disarm` mode does not apply that check in the mission node. The archived controller has its own altitude guard.

## Bringup order in the course environment

Start the matching PX4 SITL/Gazebo environment first. Source ROS 2 and the built workspace in each terminal. Then launch the following in separate terminals:

```bash
ros2 launch gazebo_env_setup topic_bridge.launch.py
ros2 launch gazebo_env_setup pose_tf_broadcaster.launch.py
ros2 launch gazebo_env_setup controller_setup.launch.py
```

`controller_setup.launch.py` starts the DDS agent on UDP port 8888 and then the controller. Do not start another agent through `combined_launch.py` at the same time.

Inspect connections before starting the mission:

```bash
ros2 topic info /command/pose
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /command/land
```

After confirming coordinate axes **and origins**, an interface-check launch for the included ENU-converting controller is:

```bash
ros2 launch uav_waypoint_mission waypoint_mission.launch.py \
  command_pose_mode:=map_enu finish_action:=hold
```

The node waits for TF and then a start command:

```bash
ros2 topic pub --once /uav_waypoint/start std_msgs/msg/Bool "{data: true}"
ros2 topic echo /uav_waypoint/status
```

Use `/uav_waypoint/stop` to request a current-position hold when a pose is available, and `/uav_waypoint/reset` to reset the waypoint index. Reset does not reset the latched coordinate origin.

## Waypoint format and defaults

[Archived route](../src/uav_waypoint_mission/config/waypoints.csv): nine numeric rows, with a commented header.

```text
x,y,z,yaw,gimbal_pitch,task
```

- At least three numeric values are required. Missing yaw inherits the preceding yaw, initially zero.
- The fifth value is optional gimbal pitch; publication defaults to disabled.
- The sixth numeric `task` field is present for coordination but does not select behaviors in this node.
- Every nonempty field is parsed as a number before the first five are used. Text in the `task` field causes the row to be skipped.
- In `yaw_unit=auto`, magnitudes greater than `2π + 0.001` are interpreted as degrees; smaller values are radians. An explicit unit is preferable for new routes with small degree values. `yaw_unit` is a node parameter, not an exposed argument in the archived launch file.
- Default mission loop: 20 Hz. Default arrival tolerance: 1.0 m. These are configuration values, not measured flight accuracy.

The [original package README](../src/uav_waypoint_mission/README.md) is retained for provenance. Its controller description predates some integration details visible in the archived C++ source; use the source-based distinctions above when reproducing the snapshot.
