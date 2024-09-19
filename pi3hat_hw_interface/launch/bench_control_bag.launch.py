import os
 
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler,DeclareLaunchArgument, TimerAction, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command,LaunchConfiguration,PathJoinSubstitution,PythonExpression,TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterFile
import subprocess
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown
def generate_launch_description():
        kin_chain_value = LaunchConfiguration("kin_chain")
        cu_param_value = LaunchConfiguration("cu_param")
        bag_name_value = LaunchConfiguration("bag_name")
        
        
        kin_chain_arg = DeclareLaunchArgument("kin_chain",default_value="Example")
        cu_param_arg = DeclareLaunchArgument("cu_param",default_value="Example")
        bag_name_arg = DeclareLaunchArgument("bag_name",default_value="Example")
        
        start_controller = ExecuteProcess(
                cmd=["ros2 control load_controller state_broadcaster --set-state active"],
                shell=True,
                output='screen',
        )
        path_name = PathJoinSubstitution(["/home/mulsbc/Benchmark", kin_chain_value, cu_param_value, bag_name_value])
        # start_bag = ExecuteProcess(
        #         cmd=["ros2", "bag", "record" ,'-o',path_name,'-a'],
        #         shell=True,
        #         output='screen',
        # )
        start_bag = Node(
                package="pi3hat_hw_interface",
                executable="pi3hat_hw_interface_record",
                 parameters=[{"path": path_name}]

        )
        exec_bag  = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_controller,
            on_exit=[start_bag],
        )

    )
#        dealy_exec_all = TimerAction(
#        period=5.0,
#        actions=[start_controller
#        ]
#    )
        end_bench = TimerAction(        period=300.0,
        actions=[
            EmitEvent(event=Shutdown(reason="End Benchmarking Experiments"))
        ]
    )


        return LaunchDescription(
                        [
                                start_controller,
                                kin_chain_arg,
                                cu_param_arg,
                                exec_bag,
                                bag_name_arg,
                                end_bench,
#                                dealy_exec_all

                        ]
                        )
