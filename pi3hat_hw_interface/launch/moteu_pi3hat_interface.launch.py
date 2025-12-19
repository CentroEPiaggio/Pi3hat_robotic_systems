import os
 
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command,LaunchConfiguration,PathJoinSubstitution,PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterFile
import subprocess
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    ld = LaunchDescription()
    LaunchConfiguration("urdf_file",default="SingleJointSE.urdf.xacro")
    LaunchConfiguration("conf_file",default="single_joint.yaml")


    urdf_name_arg = DeclareLaunchArgument("urdf_file",default_value="JumpingLeg2d.urdf.xacro")
    ld.add_action(urdf_name_arg)
    conf_name_arg = DeclareLaunchArgument("conf_file",default_value="jump_leg.yaml")
    ld.add_action(conf_name_arg)

    robot_model_path = PathJoinSubstitution([
        FindPackageShare("pi3hat_hw_interface"),
        "urdf",LaunchConfiguration("urdf_file")
        ])
    robot_description = ParameterValue(
        Command(["xacro ", robot_model_path]),
        value_type=str
    )

    controller_path = PathJoinSubstitution([
        FindPackageShare("pi3hat_hw_interface"),
        "config",LaunchConfiguration("conf_file")
        ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},controller_path],
        
    )

    ld.add_action(control_node)
    return ld