import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    awdbot_description = get_package_share_directory("awdbot_description")

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                awdbot_description, "urdf", "awdbot.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("awdbot_bringup"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name"),
            "model": LaunchConfiguration("model")
        }.items()
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("awdbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("awdbot_bringup"),
            "rviz",
            "simulated_robot.rviz"
        )],
        parameters=[{"use_sim_time": True}]
    )
    
    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo,
        controller,
        rviz,
    ])