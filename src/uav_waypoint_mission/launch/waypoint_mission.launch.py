from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('uav_waypoint_mission')
    default_waypoints = os.path.join(pkg_share, 'config', 'waypoints.csv')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),

        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_waypoints,
            description='CSV file path: x,y,z,yaw,...'
        ),

        DeclareLaunchArgument(
            'auto_start',
            default_value='false',
            description='Start mission automatically when TF is ready'
        ),

        DeclareLaunchArgument(
            'arrival_tolerance',
            default_value='1.0',
            description='Waypoint arrival tolerance in meters'
        ),

        DeclareLaunchArgument(
            'uav_frame',
            default_value='x500_gimbal_0/base_link',
            description='UAV TF frame'
        ),

        DeclareLaunchArgument(
            'fallback_uav_frames',
            default_value='x500_gimbal_0',
            description='Comma-separated fallback TF frames'
        ),

        DeclareLaunchArgument(
            'command_pose_mode',
            default_value='local_ned',
            description='local_ned or map_enu'
        ),

        DeclareLaunchArgument(
            'finish_action',
            default_value='land',
            description='hold, land, disarm, land_then_disarm'
        ),

        DeclareLaunchArgument(
            'publish_gimbal_from_csv',
            default_value='false',
            description='Publish CSV 5th column to /gimbal_pitch_degree'
        ),

        Node(
            package='uav_waypoint_mission',
            executable='uav_waypoint_node',
            name='uav_waypoint_node',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'waypoints_file': LaunchConfiguration('waypoints_file'),

                'map_frame': 'map',
                'uav_frame': LaunchConfiguration('uav_frame'),
                'fallback_uav_frames': LaunchConfiguration('fallback_uav_frames'),

                'command_pose_topic': '/command/pose',
                'land_topic': '/command/land',
                'disarm_topic': '/command/disarm',
                'gimbal_pitch_topic': '/gimbal_pitch_degree',

                'status_topic': '/uav_waypoint/status',
                'current_goal_topic': '/uav_waypoint/current_goal',

                'start_topic': '/uav_waypoint/start',
                'stop_topic': '/uav_waypoint/stop',
                'reset_topic': '/uav_waypoint/reset',

                'control_rate_hz': 20.0,
                'arrival_tolerance': ParameterValue(LaunchConfiguration('arrival_tolerance'), value_type=float),
                'tf_timeout_sec': 0.05,

                'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
                'hold_on_stop': True,

                # local_ned:
                #   for offboard_control_node that directly copies pose to PX4 NED setpoint.
                # map_enu:
                #   for offboard_control_node that internally converts ENU -> NED.
                'command_pose_mode': LaunchConfiguration('command_pose_mode'),
                'local_ned_frame_id': 'local_ned',

                'yaw_unit': 'auto',
                'default_yaw': 0.0,

                # Safer defaults:
                #   hold: safest for first test
                #   land: use when /command/land subscriber exists
                #   land_then_disarm: use after confirming LAND works
                'finish_action': LaunchConfiguration('finish_action'),
                'land_command_repeat_sec': 3.0,
                'disarm_command_repeat_sec': 3.0,
                'disarm_altitude_above_home': 0.50,
                'disarm_after_land_timeout_sec': 20.0,

                'publish_gimbal_from_csv': ParameterValue(LaunchConfiguration('publish_gimbal_from_csv'), value_type=bool),
            }]
        )
    ])
