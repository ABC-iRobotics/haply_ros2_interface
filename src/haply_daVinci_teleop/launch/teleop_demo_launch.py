from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="1",
        description="Force feedback mode for the demo: 0, 1 or 2",
    )

    hotwire_demo_node = Node(
        package="haply_daVinci_teleop",
        executable="haptic_hotwire_demo",
        name="hotwire_demo",
        parameters=[{
            "mode": LaunchConfiguration("mode"),
        }],
        output="screen",
    )

    bridge_demo_node = Node(
        package="haply_daVinci_teleop",
        executable="haply_daVinci_bridge_node_jointcontrol_demo",
        name="haply_daVinci_bridge_node_jointcontrol_demo",
        output="screen",
    )

    haptic_loop_node = Node(
        package="haply_daVinci_teleop",
        executable="haptic_loop",
        name="haptic_loop",
        output="screen",
    )

    study_controller_demo_node = Node(
        package="haply_daVinci_teleop",
        executable="study_controller_demo",
        name="study_controller_demo",
        output="screen",
    )

    return LaunchDescription([
        mode_arg,
        hotwire_demo_node,
        bridge_demo_node,
        haptic_loop_node,
        study_controller_demo_node,
    ])
