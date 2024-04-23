import os
 
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command,LaunchConfiguration,PathJoinSubstitution,PythonExpression,TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterFile
import subprocess
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
        kin_chain_value = LaunchConfiguration("kin_chain")
        cu_param_value = LaunchConfiguration("cu_param")
        
        
        kin_chain_arg = DeclareLaunchArgument("kin_chain",default_value="Example")
        cu_param_arg = DeclareLaunchArgument("cu_param",default_value="Example")
        
        start_controller = ExecuteProcess(
                cmd=["ros2 control load_controller state_broadcaster --set-state active"],
                shell=True,
                output='screen',
        )
        path_name = PathJoinSubstitution(["/home/mulsbc/Benchmark", kin_chain_value, cu_param_value])
        start_bag = ExecuteProcess(
                cmd=["ros2", "bag", "record", "-a" ,'-o',path_name],
                shell=True,
                output='screen',
        )
        exec_bag  = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_controller,
            on_exit=[start_bag],
        )
    )

        return LaunchDescription(
                        [
                                start_controller,
                                kin_chain_arg,
                                cu_param_arg,
                                exec_bag,

                        ]
                        )