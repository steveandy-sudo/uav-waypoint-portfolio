from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    env_setup_dir = get_package_share_directory('gazebo_env_setup')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            name='micro_xrce_agent',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(env_setup_dir, 'launch', 'topic_bridge.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(env_setup_dir, 'launch', 'pose_tf_broadcaster.launch.py')
            )
        )
    ])
