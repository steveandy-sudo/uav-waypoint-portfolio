from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

import os

def generate_launch_description():
    env_setup_dir = get_package_share_directory('gazebo_env_setup')
    use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time', default_value='true', description='Use /clock time if true')

    topic_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(env_setup_dir, 'launch', 'topic_bridge.launch.py')
        )
    )

    pose_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(env_setup_dir, 'launch', 'pose_tf_broadcaster.launch.py')
        )
    )

    controller_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(env_setup_dir, 'launch', 'controller_setup.launch.py')
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        topic_bridge,
        pose_tf,
        controller_setup,
    ])
