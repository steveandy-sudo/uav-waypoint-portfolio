from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time', default_value='true', description='Use /clock time if true')

    # ── Micro XRCE-DDS Agent (가장 먼저) ──────────────────
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='micro_xrce_agent',
        output='screen'
    )

    # ── 나머지 노드 묶음 (5초 뒤 시작) ─────────────────────
    delayed_nodes = TimerAction(
        period=5.0,      # ⬅︎ 여기서 지연 시간(초) 조정
        actions=[
            Node(
                package='px4_ros_com',
                executable='offboard_control',
                name='offboard_control',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ]
            ),
        ]
    )

    return LaunchDescription([
        micro_xrce_agent,
        use_sim_time_arg,
        delayed_nodes
    ])
