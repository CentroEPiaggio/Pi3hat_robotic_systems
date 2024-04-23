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

    kin_chain_value = LaunchConfiguration("kin_chain",default="2")
    cu_param_value = LaunchConfiguration("cu_param",default="4")
    use_imu_value = LaunchConfiguration("use_imu",default="0")
    main_t_value = LaunchConfiguration("main_t",default="800000")
    can_t_value = LaunchConfiguration("can_t",default="6000")
    rec_t_value = LaunchConfiguration("rec_t",default="1000")

    kin_chain_arg = DeclareLaunchArgument("kin_chain",default_value="Example")
    cu_param_arg = DeclareLaunchArgument("cu_param",default_value="Example")
    use_imu_arg = DeclareLaunchArgument("use_imu",default_value="0")
    main_t_arg = DeclareLaunchArgument("main_t",default_value="600000")
    can_t_arg = DeclareLaunchArgument("can_t",default_value="1000")
    rec_t_arg = DeclareLaunchArgument("rec_t",default_value="1000")


  


    start_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path("pi3hat_hw_interface"), "launch", "benchmark_test.launch.py")
        ),
        launch_arguments= {'use_imu' : use_imu_value,'main_t' :  main_t_value,'can_t' : can_t_value,'rec_t' : rec_t_value}.items()
    )
    start_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path("pi3hat_hw_interface"), "launch", "bench_control_bag.launch.py")
        ),
        launch_arguments= {'kin_chain' : kin_chain_value, "cu_param" : cu_param_value}.items()
    )
    # TODO add end launch process after n seconds
    return LaunchDescription(
        [
            start_interface,
            start_controller,
            kin_chain_arg,
            cu_param_arg,
            use_imu_arg,
            main_t_arg,
            can_t_arg,
            rec_t_arg
        ]
    )