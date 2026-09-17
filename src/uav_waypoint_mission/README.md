# uav_waypoint_mission

ROS 2 Humble package for running a simple UAV waypoint mission in PX4 SITL and Gazebo Sim.

This package provides one node, `uav_waypoint_node`. It reads waypoint coordinates from a CSV file, tracks the UAV pose through TF, publishes pose commands for an existing offboard control node, and sends optional land/disarm commands when the mission is complete.

## Package Role

`uav_waypoint_mission` is the waypoint mission layer.

It does not start PX4, Gazebo, or the offboard controller by itself. Those systems must already be running and must provide:

- TF from `map` to the UAV frame, default `x500_gimbal_0/base_link`
- a subscriber for `/command/pose`
- optional subscribers for `/command/land` and `/command/disarm`

In the full project runtime, the bridge still needs to be launched. The point is that this package assumes the bridge and offboard control layer are already active instead of owning them internally.

Typical runtime stack:

```text
Gazebo / PX4 SITL
  -> bridge and TF layer
  -> ROS 2 topics / TF
  -> uav_waypoint_node
  -> /command/pose
  -> offboard control node
  -> PX4 setpoints
```

If the bridge is responsible for `/clock`, TF, PX4 DDS topics, or Gazebo-to-ROS data, start it before starting the waypoint mission.

## Node

### `uav_waypoint_node`

Responsibilities:

- Load waypoints from `config/waypoints.csv`
- Wait until the UAV TF is available
- Publish the current waypoint target to `/command/pose`
- Advance to the next waypoint when the UAV is within `arrival_tolerance`
- Publish mission state to `/uav_waypoint/status`
- Publish the active goal to `/uav_waypoint/current_goal` for debugging or RViz
- React to start, stop, and reset topics
- Optionally publish gimbal pitch from the CSV fifth column
- Optionally send land or disarm commands after the final waypoint

## Topics

Published:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/command/pose` | `geometry_msgs/msg/PoseStamped` | Target pose for the offboard control node |
| `/command/land` | `std_msgs/msg/Bool` | Land command |
| `/command/disarm` | `std_msgs/msg/Bool` | Disarm command |
| `/gimbal_pitch_degree` | `std_msgs/msg/Float32` | Optional gimbal pitch command |
| `/uav_waypoint/status` | `std_msgs/msg/String` | Mission state text |
| `/uav_waypoint/current_goal` | `geometry_msgs/msg/PoseStamped` | Current waypoint in map frame |

Subscribed:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/uav_waypoint/start` | `std_msgs/msg/Bool` | Start mission when `true` |
| `/uav_waypoint/stop` | `std_msgs/msg/Bool` | Stop mission and hold current pose when possible |
| `/uav_waypoint/reset` | `std_msgs/msg/Bool` | Reset mission index and return to idle |

## Waypoint CSV

Default file:

```text
config/waypoints.csv
```

Format:

```text
x,y,z,yaw,gimbal_pitch,task
```

Only the first five columns are used by the current node:

- `x`, `y`, `z`: waypoint position in the map frame
- `yaw`: target yaw, interpreted automatically as radians or degrees
- `gimbal_pitch`: optional pitch command in degrees

The sixth `task` column is kept in the file for team coordination, but the current code does not branch on it.

## Build

From the ROS 2 workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_waypoint_mission
source install/setup.bash
```

## Run

Start the waypoint mission node:

```bash
ros2 launch uav_waypoint_mission waypoint_mission.launch.py
```

The default launch configuration waits for TF and then stays idle until a start command is received.

## Full Project Bringup With `utilities_pkg`

In the team project, the bridge and offboard control pieces are provided by `utilities_pkg`.

Relevant files in that package:

| File | Role |
| --- | --- |
| `gazebo_env_setup/launch/topic_bridge.launch.py` | Starts `ros_gz_bridge parameter_bridge` for `/clock`, model poses, camera, and lidar topics |
| `gazebo_env_setup/launch/pose_tf_broadcaster.launch.py` | Converts bridged Gazebo model pose topics into ROS TF |
| `gazebo_env_setup/launch/controller_setup.launch.py` | Starts `MicroXRCEAgent` and the PX4 offboard control node |
| `px4_ros_com/src/examples/offboard/offboard_control.cpp` | Subscribes to `/command/pose`, `/command/disarm`, and `/gimbal_pitch_degree` |

Recommended bringup order:

```bash
# 1. Source ROS 2 and both workspaces.
source /opt/ros/humble/setup.bash
source /home/junghun/ws_autonomous_driving_platform/utilities_pkg/install/setup.bash
source /home/junghun/ws_autonomous_driving_platform/uav_waypoint_ws/install/setup.bash

# 2. Start PX4 SITL and Gazebo in the project environment.

# 3. Start the Gazebo-to-ROS bridge for the team simulation.
ros2 launch gazebo_env_setup topic_bridge.launch.py

# 4. Convert bridged model poses into TF.
ros2 launch gazebo_env_setup pose_tf_broadcaster.launch.py

# 5. Start Micro XRCE-DDS Agent and the offboard control node.
ros2 launch gazebo_env_setup controller_setup.launch.py

# 6. Start this waypoint mission node.
ros2 launch uav_waypoint_mission waypoint_mission.launch.py finish_action:=hold
```

For this waypoint package, the important checks are:

```bash
ros2 topic info /command/pose
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
ros2 topic echo /uav_waypoint/status
```

`/command/pose` should have a subscriber from the offboard control node, and `tf2_echo` should show the UAV transform. If either one is missing, check the `utilities_pkg` bridge, TF broadcaster, and offboard control launch first.

The current `utilities_pkg` offboard control code subscribes to `/command/pose`, `/command/disarm`, and `/gimbal_pitch_degree`. A `/command/land` subscriber was not found in that code, so `finish_action:=hold` is the recommended first integration setting unless the team adds or confirms a land-command subscriber.

Start the mission:

```bash
ros2 topic pub --once /uav_waypoint/start std_msgs/msg/Bool "{data: true}"
```

Stop and hold:

```bash
ros2 topic pub --once /uav_waypoint/stop std_msgs/msg/Bool "{data: true}"
```

Reset:

```bash
ros2 topic pub --once /uav_waypoint/reset std_msgs/msg/Bool "{data: true}"
```

Monitor status:

```bash
ros2 topic echo /uav_waypoint/status
```

## Useful Launch Arguments

Auto-start when TF becomes available:

```bash
ros2 launch uav_waypoint_mission waypoint_mission.launch.py auto_start:=true
```

Use a custom waypoint CSV:

```bash
ros2 launch uav_waypoint_mission waypoint_mission.launch.py waypoints_file:=/absolute/path/to/waypoints.csv
```

Keep the final waypoint instead of landing:

```bash
ros2 launch uav_waypoint_mission waypoint_mission.launch.py finish_action:=hold
```

Publish gimbal pitch commands from the CSV fifth column:

```bash
ros2 launch uav_waypoint_mission waypoint_mission.launch.py publish_gimbal_from_csv:=true
```

Change arrival tolerance:

```bash
ros2 launch uav_waypoint_mission waypoint_mission.launch.py arrival_tolerance:=0.5
```

## Coordinate Mode

Default:

```bash
command_pose_mode:=local_ned
```

Use this when the offboard control node directly copies `/command/pose` into a PX4 NED setpoint.

Alternative:

```bash
command_pose_mode:=map_enu
```

Use this when the offboard control node already converts ENU map coordinates to NED internally.

## Safety Notes

For first integration tests, use:

```bash
ros2 launch uav_waypoint_mission waypoint_mission.launch.py finish_action:=hold
```

After confirming that `/command/land` is connected and works correctly, use:

```bash
ros2 launch uav_waypoint_mission waypoint_mission.launch.py finish_action:=land
```

Use `finish_action:=land_then_disarm` only after landing behavior has been verified.
