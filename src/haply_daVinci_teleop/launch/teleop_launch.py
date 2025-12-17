from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='0',
        description='Force feedback mode: 0, 1 or 2'
    )

    hotwire_node = Node(
        package='haply_daVinci_teleop',
        executable='haptic_hotwire',
        name='hotwire',
        parameters=[{
            'mode': LaunchConfiguration('mode')
        }],
        output='screen'
    )

    bridge_node = Node(
        package='haply_daVinci_teleop',
        executable='haply_daVinci_bridge_node_jointcontrol',
        name='haply_daVinci_bridge_node_jointcontrol',
        output='screen'
    )

    haptic_loop_node = Node(
        package='haply_daVinci_teleop',
        executable='haptic_loop',
        name='haptic_loop',
        output='screen'
    )

    return LaunchDescription([
        mode_arg,
        hotwire_node,
        bridge_node,
        haptic_loop_node,
    ])
