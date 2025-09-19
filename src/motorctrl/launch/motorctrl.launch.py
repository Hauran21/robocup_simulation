from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    node_diffdrivectrl = Node(
        package='motorctrl',
        executable='diffdrive',
        name='diffdrivectrl',
        output='screen',
        parameters=[]
    )

    node_dropctrl = Node(
        package='motorctrl',
        executable='dropctrl',
        name='dropctrl',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(node_diffdrivectrl)
    # ld.add_action(node_dropctrl)
    
    return ld