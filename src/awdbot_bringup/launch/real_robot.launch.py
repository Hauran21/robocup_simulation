import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import yaml
def generate_launch_description():

    awdbot_description_path = get_package_share_directory("awdbot_description")
    #awdbot_bringup_path = get_package_share_directory("awdbot_bringup")
    awdbot_controller_path = get_package_share_directory("awdbot_controller")
    #motorctrl_path = get_package_share_directory("motorctrl")

    diffdrivectrl_config_path = os.path.join(awdbot_controller_path, "config", "diffdrivectrl.yaml")
    # YAML laden
    with open(diffdrivectrl_config_path, 'r') as f:
        data = yaml.safe_load(f) or {}

    # Parameter aus deinem Node-Block ziehen
    dd = (data.get('diffdrivectrl') or {}).get('ros__parameters') or {}

    wheel_radius     = str(dd.get('wheel_radius', 0.04))
    wheel_separation = str(dd.get('wheel_separation', 0.18))
    port_name        = str(dd.get('uart_name', '/dev/ttyACM0'))
    baud_rate        = str(dd.get('uart_baudrate', 57600))
    motor_ids        = dd.get('motor_ids', [1, 2, 3, 4])

    larg_urdf_path = DeclareLaunchArgument(name="urdf_path", 
                                           default_value=os.path.join(awdbot_description_path, "urdf", "awdbot.urdf.xacro"),
                                           description="Pfad zur URDF-Datei des Roboters")

    larg_use_simple_controller = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="False",
    )

    robot_description = ParameterValue(Command(["xacro ",
                                                LaunchConfiguration("urdf_path"), 
                                                " is_sim:=false",
                                                f" port_name:={port_name}",
                                                f" baud_rate:={baud_rate}",
                                                f" dxl1_id:={motor_ids[0]}",
                                                f" dxl2_id:={motor_ids[1]}",
                                                f" dxl3_id:={motor_ids[2]}",
                                                f" dxl4_id:={motor_ids[3]}",
                                                f" wheel_radius:={wheel_radius}",
                                                f" chassis_width:={wheel_separation}"]), value_type=str)

    controller_config = os.path.join(awdbot_controller_path,"config","awdbot_controllers.yaml")
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            #{"robot_description": robot_description},
            controller_config,
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description")
        ]
    )

    controller_diffdrive_spawn = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("awdbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": LaunchConfiguration("use_simple_controller"),
            "use_sim_time": "False",
        }.items(),
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            awdbot_description_path,
            "rviz",
            "display.rviz"
        )]
    )
    
    return LaunchDescription([
        larg_urdf_path,
        larg_use_simple_controller,
        robot_state_publisher_node,
        controller_manager,
        controller_diffdrive_spawn,
        node_rviz
    ])