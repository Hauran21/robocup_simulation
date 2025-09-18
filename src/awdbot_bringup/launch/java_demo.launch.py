import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():

    arg_use_fake_nodes = DeclareLaunchArgument(
        name="use_fake_nodes", default_value="True",
        description="use fake notes"
    )

    group_fake = GroupAction(
        condition=IfCondition(arg_use_fake_nodes),
        actions=[
            Node(
                package="motorctrl",
                executable="dummy",
                name="diffdrivectrl",
                output="screen",
            ),
            Node(
                package="cameractrl",
                executable="dummy",
                name="camera_left",
                output="screen",
            ),
            Node(
                package="cameractrl",
                executable="dummy",
                name="camera_right",
                output="screen",
            )
        ]
    )

    group_real= GroupAction(
        condition=UnlessCondition(arg_use_fake_nodes),
        actions=[
            Node(
                package="motorctrl",
                executable="diffdrive_drop",
                name="diffdrivectrl",
                output="screen",
            ),
            Node(
                package="cameractrl",
                executable="openmv",
                name="camera_left",
                output="screen",
            )
        ]
    )

    launch_ledctrl = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ledctrl"),
            "launch",
            "ledctrl.launch.py"
        )
    )

    ld = LaunchDescription()
    ld.add_action(arg_use_fake_nodes)
    ld.add_action(group_fake)
    ld.add_action(group_real)
    ld.add_action(launch_ledctrl)

    return ld