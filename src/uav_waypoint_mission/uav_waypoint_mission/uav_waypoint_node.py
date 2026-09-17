#!/usr/bin/env python3

import csv
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String, Float32

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float
    gimbal_pitch: Optional[float] = None
    raw_column_count: int = 0


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float):
    """Create geometry_msgs Quaternion-like tuple from yaw only."""
    half = yaw * 0.5
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw


def quaternion_to_yaw(q) -> float:
    """Extract yaw from geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def distance_3d(a: Tuple[float, float, float, float],
                wp: Waypoint) -> float:
    dx = a[0] - wp.x
    dy = a[1] - wp.y
    dz = a[2] - wp.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class UAVWaypointMission(Node):
    """
    Simple waypoint mission node.

    It reads map-frame waypoints from CSV and publishes PoseStamped commands
    to /command/pose for the existing PX4 offboard_control_node.

    command_pose_mode:
      - local_ned:
          Use this when offboard_control_node directly copies msg.pose.position
          into px4_msgs/TrajectorySetpoint without ENU->NED conversion.
          The node converts map ENU-like coordinates into local NED-like values.
      - map_enu:
          Use this when offboard_control_node internally converts ENU to NED.
          The node publishes the CSV map coordinates directly.
    """

    def __init__(self):
        super().__init__('uav_waypoint_node')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        # Do not declare 'use_sim_time' here.
        # ROS 2 / launch may already declare it internally, and declaring it again
        # causes ParameterAlreadyDeclaredException in rclpy.

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('uav_frame', 'x500_gimbal_0/base_link')
        self.declare_parameter('fallback_uav_frames', 'x500_gimbal_0')

        self.declare_parameter('command_pose_topic', '/command/pose')
        self.declare_parameter('land_topic', '/command/land')
        self.declare_parameter('disarm_topic', '/command/disarm')
        self.declare_parameter('gimbal_pitch_topic', '/gimbal_pitch_degree')

        self.declare_parameter('status_topic', '/uav_waypoint/status')
        self.declare_parameter('current_goal_topic', '/uav_waypoint/current_goal')

        self.declare_parameter('start_topic', '/uav_waypoint/start')
        self.declare_parameter('stop_topic', '/uav_waypoint/stop')
        self.declare_parameter('reset_topic', '/uav_waypoint/reset')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('arrival_tolerance', 1.0)
        self.declare_parameter('tf_timeout_sec', 0.05)

        self.declare_parameter('auto_start', False)
        self.declare_parameter('hold_on_stop', True)

        # "local_ned" or "map_enu"
        self.declare_parameter('command_pose_mode', 'local_ned')
        self.declare_parameter('local_ned_frame_id', 'local_ned')

        # "auto", "rad", "deg"
        self.declare_parameter('yaw_unit', 'auto')
        self.declare_parameter('default_yaw', 0.0)

        # Finish behavior: hold, land, disarm, land_then_disarm
        self.declare_parameter('finish_action', 'land')
        self.declare_parameter('land_command_repeat_sec', 3.0)
        self.declare_parameter('disarm_command_repeat_sec', 3.0)
        self.declare_parameter('disarm_altitude_above_home', 0.50)
        self.declare_parameter('disarm_after_land_timeout_sec', 20.0)

        # Optional gimbal control from CSV 5th column
        self.declare_parameter('publish_gimbal_from_csv', False)

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.map_frame = self.get_parameter('map_frame').value
        self.uav_frame = self.get_parameter('uav_frame').value
        fallback = self.get_parameter('fallback_uav_frames').value
        self.fallback_uav_frames = [
            item.strip() for item in fallback.split(',') if item.strip()
        ]

        self.command_pose_topic = self.get_parameter('command_pose_topic').value
        self.land_topic = self.get_parameter('land_topic').value
        self.disarm_topic = self.get_parameter('disarm_topic').value
        self.gimbal_pitch_topic = self.get_parameter('gimbal_pitch_topic').value

        self.status_topic = self.get_parameter('status_topic').value
        self.current_goal_topic = self.get_parameter('current_goal_topic').value

        self.start_topic = self.get_parameter('start_topic').value
        self.stop_topic = self.get_parameter('stop_topic').value
        self.reset_topic = self.get_parameter('reset_topic').value

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.arrival_tolerance = float(self.get_parameter('arrival_tolerance').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)

        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.hold_on_stop = bool(self.get_parameter('hold_on_stop').value)

        self.command_pose_mode = self.get_parameter('command_pose_mode').value
        self.local_ned_frame_id = self.get_parameter('local_ned_frame_id').value

        self.yaw_unit = self.get_parameter('yaw_unit').value
        self.default_yaw = float(self.get_parameter('default_yaw').value)

        self.finish_action = self.get_parameter('finish_action').value
        self.land_command_repeat_sec = float(
            self.get_parameter('land_command_repeat_sec').value)
        self.disarm_command_repeat_sec = float(
            self.get_parameter('disarm_command_repeat_sec').value)
        self.disarm_altitude_above_home = float(
            self.get_parameter('disarm_altitude_above_home').value)
        self.disarm_after_land_timeout_sec = float(
            self.get_parameter('disarm_after_land_timeout_sec').value)

        self.publish_gimbal_from_csv = bool(
            self.get_parameter('publish_gimbal_from_csv').value)

        if self.control_rate_hz <= 0.0:
            self.get_logger().warn('control_rate_hz <= 0. Using 20 Hz.')
            self.control_rate_hz = 20.0

        # ------------------------------------------------------------------
        # TF
        # ------------------------------------------------------------------
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------
        # Publishers / Subscribers
        # ------------------------------------------------------------------
        self.pose_pub = self.create_publisher(
            PoseStamped, self.command_pose_topic, 10)
        self.land_pub = self.create_publisher(
            Bool, self.land_topic, 10)
        self.disarm_pub = self.create_publisher(
            Bool, self.disarm_topic, 10)
        self.gimbal_pub = self.create_publisher(
            Float32, self.gimbal_pitch_topic, 10)

        self.status_pub = self.create_publisher(
            String, self.status_topic, 10)
        self.current_goal_pub = self.create_publisher(
            PoseStamped, self.current_goal_topic, 10)

        self.start_sub = self.create_subscription(
            Bool, self.start_topic, self.start_callback, 10)
        self.stop_sub = self.create_subscription(
            Bool, self.stop_topic, self.stop_callback, 10)
        self.reset_sub = self.create_subscription(
            Bool, self.reset_topic, self.reset_callback, 10)

        # ------------------------------------------------------------------
        # Mission state
        # ------------------------------------------------------------------
        self.waypoints: List[Waypoint] = self.load_waypoints(self.waypoints_file)
        self.current_index = 0

        self.state = 'WAITING_FOR_TF'
        self.active = False
        self.hold_pose_map: Optional[Waypoint] = None

        self.origin_map: Optional[Tuple[float, float, float, float]] = None
        self.last_current_pose: Optional[Tuple[float, float, float, float]] = None

        self.finish_start_time = None
        self.last_gimbal_pitch_sent: Optional[float] = None

        period = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info('==========================================')
        self.get_logger().info('UAV waypoint mission node started')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        self.get_logger().info(f'command_pose_mode: {self.command_pose_mode}')
        self.get_logger().info(f'auto_start: {self.auto_start}')
        self.get_logger().info(f'finish_action: {self.finish_action}')
        self.get_logger().info('==========================================')

    # ----------------------------------------------------------------------
    # CSV
    # ----------------------------------------------------------------------
    def parse_yaw(self, value: float) -> float:
        if self.yaw_unit == 'deg':
            return math.radians(value)
        if self.yaw_unit == 'rad':
            return value

        # auto mode:
        # Values outside [-2pi, 2pi] are treated as degrees.
        if abs(value) > (2.0 * math.pi + 1e-3):
            return math.radians(value)
        return value

    def load_waypoints(self, path: str) -> List[Waypoint]:
        waypoints: List[Waypoint] = []

        if not path:
            self.get_logger().error('waypoints_file parameter is empty.')
            return waypoints

        self.get_logger().info(f'Reading waypoint CSV: {path}')

        last_yaw = self.default_yaw

        try:
            with open(path, 'r', newline='') as f:
                reader = csv.reader(f)

                for line_number, row in enumerate(reader, start=1):
                    if not row:
                        continue

                    cleaned = []
                    for token in row:
                        token = token.strip()
                        if token.startswith('#'):
                            break
                        if token:
                            cleaned.append(token)

                    if not cleaned:
                        continue

                    try:
                        values = [float(token) for token in cleaned]
                    except ValueError:
                        self.get_logger().warn(
                            f'Skipping non-numeric CSV line {line_number}: {row}')
                        continue

                    if len(values) < 3:
                        self.get_logger().warn(
                            f'Skipping CSV line {line_number}: need at least x,y,z')
                        continue

                    x = values[0]
                    y = values[1]
                    z = values[2]

                    if len(values) >= 4:
                        yaw = self.parse_yaw(values[3])
                        last_yaw = yaw
                    else:
                        yaw = last_yaw

                    gimbal_pitch = values[4] if len(values) >= 5 else None

                    waypoints.append(
                        Waypoint(
                            x=x,
                            y=y,
                            z=z,
                            yaw=yaw,
                            gimbal_pitch=gimbal_pitch,
                            raw_column_count=len(values),
                        )
                    )

        except FileNotFoundError:
            self.get_logger().error(f'Waypoint file not found: {path}')
        except Exception as exc:
            self.get_logger().error(f'Failed to read waypoint file: {exc}')

        for i, wp in enumerate(waypoints):
            self.get_logger().info(
                f'WP[{i}] map: x={wp.x:.2f}, y={wp.y:.2f}, z={wp.z:.2f}, '
                f'yaw={wp.yaw:.3f} rad, columns={wp.raw_column_count}'
            )

        return waypoints

    # ----------------------------------------------------------------------
    # Commands
    # ----------------------------------------------------------------------
    def start_callback(self, msg: Bool):
        if not msg.data:
            return

        if not self.waypoints:
            self.get_logger().error('Cannot start: no waypoints loaded.')
            return

        if self.state in ['FINISHED', 'COMPLETE', 'LANDING', 'DISARMING']:
            self.current_index = 0
            self.finish_start_time = None

        self.active = True
        self.state = 'RUNNING'
        self.get_logger().info('START received. Mission RUNNING.')

    def stop_callback(self, msg: Bool):
        if not msg.data:
            return

        self.active = False

        if self.hold_on_stop and self.last_current_pose is not None:
            x, y, z, yaw = self.last_current_pose
            self.hold_pose_map = Waypoint(x=x, y=y, z=z, yaw=yaw)
            self.state = 'HOLD'
            self.get_logger().warn('STOP received. Holding current pose.')
        else:
            self.state = 'STOPPED'
            self.get_logger().warn('STOP received. No hold pose available.')

    def reset_callback(self, msg: Bool):
        if not msg.data:
            return

        self.current_index = 0
        self.active = False
        self.hold_pose_map = None
        self.finish_start_time = None
        self.state = 'IDLE'
        self.get_logger().info('RESET received. Mission reset to IDLE.')

    # ----------------------------------------------------------------------
    # TF / Pose
    # ----------------------------------------------------------------------
    def lookup_current_pose_map(self) -> Optional[Tuple[float, float, float, float]]:
        frames_to_try = [self.uav_frame] + self.fallback_uav_frames

        for frame in frames_to_try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec)
                )

                t = transform.transform.translation
                q = transform.transform.rotation
                yaw = quaternion_to_yaw(q)

                return (t.x, t.y, t.z, yaw)

            except TransformException:
                continue

        self.get_logger().warn(
            f'TF lookup failed: {self.map_frame} -> '
            f'{self.uav_frame} or fallbacks {self.fallback_uav_frames}',
            throttle_duration_sec=2.0
        )
        return None

    def ensure_origin(self, current_pose_map: Tuple[float, float, float, float]):
        if self.origin_map is not None:
            return

        self.origin_map = current_pose_map
        ox, oy, oz, oyaw = self.origin_map

        self.get_logger().info(
            'Local NED origin locked from current map pose: '
            f'x={ox:.3f}, y={oy:.3f}, z={oz:.3f}, yaw={oyaw:.3f}'
        )

    # ----------------------------------------------------------------------
    # Publishing
    # ----------------------------------------------------------------------
    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_gimbal_if_needed(self, wp: Waypoint):
        if not self.publish_gimbal_from_csv:
            return
        if wp.gimbal_pitch is None:
            return

        # Avoid spamming the same value too aggressively.
        if self.last_gimbal_pitch_sent is not None:
            if abs(self.last_gimbal_pitch_sent - wp.gimbal_pitch) < 1e-3:
                return

        msg = Float32()
        msg.data = float(wp.gimbal_pitch)
        self.gimbal_pub.publish(msg)
        self.last_gimbal_pitch_sent = wp.gimbal_pitch

        self.get_logger().info(
            f'Published gimbal pitch from CSV: {wp.gimbal_pitch:.2f} deg'
        )

    def make_command_pose(self, wp: Waypoint) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        mode = self.command_pose_mode.lower()

        if mode in ['map', 'map_enu', 'enu']:
            # Use this mode when offboard_control_node performs ENU -> NED conversion.
            cmd_x = wp.x
            cmd_y = wp.y
            cmd_z = wp.z
            cmd_yaw = wp.yaw
            msg.header.frame_id = self.map_frame

        elif mode in ['local_ned', 'ned', 'px4_ned_direct']:
            # Use this mode when offboard_control_node directly copies
            # /command/pose into px4_msgs/TrajectorySetpoint.
            if self.origin_map is None:
                raise RuntimeError('origin_map is not set yet.')

            ox, oy, oz, _ = self.origin_map

            # map/Gazebo ENU-like -> PX4 local NED-like
            # This matches the transform pattern used by the existing uav_final.cpp:
            # local_ned.x = map.y - origin.y
            # local_ned.y = map.x - origin.x
            # local_ned.z = -(map.z - origin.z)
            cmd_x = wp.y - oy
            cmd_y = wp.x - ox
            cmd_z = -(wp.z - oz)

            # ENU yaw -> NED yaw
            cmd_yaw = normalize_angle(math.pi / 2.0 - wp.yaw)
            msg.header.frame_id = self.local_ned_frame_id

        else:
            self.get_logger().warn(
                f'Unknown command_pose_mode={self.command_pose_mode}. '
                'Falling back to map_enu.'
            )
            cmd_x = wp.x
            cmd_y = wp.y
            cmd_z = wp.z
            cmd_yaw = wp.yaw
            msg.header.frame_id = self.map_frame

        msg.pose.position.x = float(cmd_x)
        msg.pose.position.y = float(cmd_y)
        msg.pose.position.z = float(cmd_z)

        qx, qy, qz, qw = yaw_to_quaternion(cmd_yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        return msg

    def publish_waypoint_command(self, wp: Waypoint):
        try:
            cmd = self.make_command_pose(wp)
        except RuntimeError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=1.0)
            return

        self.pose_pub.publish(cmd)

        # Publish current goal in map frame for debugging / RViz.
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.map_frame
        goal_msg.pose.position.x = wp.x
        goal_msg.pose.position.y = wp.y
        goal_msg.pose.position.z = wp.z

        qx, qy, qz, qw = yaw_to_quaternion(wp.yaw)
        goal_msg.pose.orientation.x = qx
        goal_msg.pose.orientation.y = qy
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw

        self.current_goal_pub.publish(goal_msg)
        self.publish_gimbal_if_needed(wp)

    def publish_land(self):
        msg = Bool()
        msg.data = True
        self.land_pub.publish(msg)

    def publish_disarm(self):
        msg = Bool()
        msg.data = True
        self.disarm_pub.publish(msg)

    # ----------------------------------------------------------------------
    # Main loop
    # ----------------------------------------------------------------------
    def timer_callback(self):
        current_pose = self.lookup_current_pose_map()
        if current_pose is None:
            self.state = 'WAITING_FOR_TF'
            self.publish_status('WAITING_FOR_TF')
            return

        self.last_current_pose = current_pose
        self.ensure_origin(current_pose)

        if self.state == 'WAITING_FOR_TF':
            self.state = 'RUNNING' if self.auto_start else 'IDLE'
            self.active = self.auto_start
            self.get_logger().info(f'TF ready. State -> {self.state}')

        # IDLE: do not publish /command/pose until explicit start.
        if self.state == 'IDLE':
            self.publish_status(
                f'IDLE | waypoints={len(self.waypoints)} | '
                f'publish /uav_waypoint/start true to start'
            )
            return

        # STOPPED: do not publish pose.
        if self.state == 'STOPPED':
            self.publish_status('STOPPED')
            return

        # HOLD: keep current pose command alive.
        if self.state == 'HOLD':
            if self.hold_pose_map is not None:
                self.publish_waypoint_command(self.hold_pose_map)
            self.publish_status('HOLD')
            return

        # RUNNING
        if self.state == 'RUNNING':
            if not self.waypoints:
                self.publish_status('ERROR_NO_WAYPOINTS')
                return

            if self.current_index >= len(self.waypoints):
                self.state = 'FINISHED'
                self.finish_start_time = self.get_clock().now()
                return

            current_wp = self.waypoints[self.current_index]
            self.publish_waypoint_command(current_wp)

            dist = distance_3d(current_pose, current_wp)

            self.publish_status(
                f'RUNNING | index={self.current_index + 1}/{len(self.waypoints)} '
                f'| dist={dist:.2f} m'
            )

            self.get_logger().info(
                f'WP {self.current_index + 1}/{len(self.waypoints)} '
                f'distance={dist:.2f} m',
                throttle_duration_sec=2.0
            )

            if dist <= self.arrival_tolerance:
                self.get_logger().info(
                    f'Arrived at waypoint {self.current_index + 1}/'
                    f'{len(self.waypoints)}. dist={dist:.2f} m'
                )

                self.current_index += 1

                if self.current_index >= len(self.waypoints):
                    last_wp = self.waypoints[-1]
                    self.hold_pose_map = Waypoint(
                        x=last_wp.x, y=last_wp.y, z=last_wp.z, yaw=last_wp.yaw
                    )
                    self.finish_start_time = self.get_clock().now()

                    action = self.finish_action.lower()
                    if action == 'hold':
                        self.state = 'COMPLETE'
                        self.get_logger().info('Mission complete. Holding final waypoint.')
                    elif action == 'land':
                        self.state = 'LANDING'
                        self.get_logger().info('Mission complete. Sending LAND command.')
                    elif action == 'disarm':
                        self.state = 'DISARMING'
                        self.get_logger().warn(
                            'Mission complete. Sending DISARM command. '
                            'This is only safe near ground.'
                        )
                    elif action == 'land_then_disarm':
                        self.state = 'LANDING'
                        self.get_logger().info(
                            'Mission complete. Sending LAND then DISARM near ground.'
                        )
                    else:
                        self.state = 'COMPLETE'
                        self.get_logger().warn(
                            f'Unknown finish_action={self.finish_action}. Holding final waypoint.'
                        )
            return

        # LANDING
        if self.state == 'LANDING':
            if self.hold_pose_map is not None:
                # Keep publishing final pose until offboard_control accepts LAND.
                self.publish_waypoint_command(self.hold_pose_map)

            now = self.get_clock().now()
            elapsed = 0.0
            if self.finish_start_time is not None:
                elapsed = (now - self.finish_start_time).nanoseconds * 1e-9

            if elapsed <= self.land_command_repeat_sec:
                self.publish_land()

            self.publish_status(f'LANDING | elapsed={elapsed:.1f}s')

            if self.finish_action.lower() == 'land_then_disarm':
                if self.origin_map is not None:
                    home_z = self.origin_map[2]
                    current_z = current_pose[2]
                    near_ground = current_z <= home_z + self.disarm_altitude_above_home

                    if near_ground:
                        self.get_logger().info(
                            f'Near home altitude: current_z={current_z:.2f}, '
                            f'home_z={home_z:.2f}. Sending DISARM.'
                        )
                        self.state = 'DISARMING'
                        self.finish_start_time = self.get_clock().now()

                    elif elapsed > self.disarm_after_land_timeout_sec:
                        self.get_logger().warn(
                            'LAND command sent, but altitude is still high. '
                            'Not disarming for safety. State -> COMPLETE_HOLD.'
                        )
                        self.state = 'COMPLETE'

            elif elapsed > self.land_command_repeat_sec:
                self.state = 'COMPLETE'
                self.get_logger().info('LAND command repeated. State -> COMPLETE.')

            return

        # DISARMING
        if self.state == 'DISARMING':
            now = self.get_clock().now()
            elapsed = 0.0
            if self.finish_start_time is not None:
                elapsed = (now - self.finish_start_time).nanoseconds * 1e-9

            if elapsed <= self.disarm_command_repeat_sec:
                self.publish_disarm()

            self.publish_status(f'DISARMING | elapsed={elapsed:.1f}s')

            if elapsed > self.disarm_command_repeat_sec:
                self.state = 'COMPLETE'
                self.get_logger().info('DISARM command repeated. State -> COMPLETE.')

            return

        # COMPLETE: hold final pose unless land/disarm already stopped offboard.
        if self.state == 'COMPLETE':
            if self.hold_pose_map is not None and self.finish_action.lower() == 'hold':
                self.publish_waypoint_command(self.hold_pose_map)

            self.publish_status('COMPLETE')
            self.get_logger().info(
                'Mission COMPLETE. Node is idling.',
                throttle_duration_sec=10.0
            )
            return


def main(args=None):
    rclpy.init(args=args)
    node = UAVWaypointMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
