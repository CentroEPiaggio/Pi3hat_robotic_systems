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


    #get solo12 urdf with system HW interface 



    moteus_pi3hat_pkg_path = get_package_share_path("pi3hat_hw_interface")
   
    urdf_name_val = LaunchConfiguration("urdf_file")
    conf_name_val = LaunchConfiguration("conf_file")

    urdf_name_arg = DeclareLaunchArgument("urdf_file",default_value="8DOF_mul_off.urdf.xacro")
    conf_name_arg = DeclareLaunchArgument("conf_file",default_value="test_config.yaml")
   
    
    moteus_pi3hat_path = PathJoinSubstitution([
        FindPackageShare("pi3hat_hw_interface"),
        "urdf",
       urdf_name_val
       ]
    )
    moteus_pi3hat_pkg_path = PathJoinSubstitution([
        FindPackageShare("pi3hat_hw_interface")
       ]
    )
    # a = Command([ "/home/jacopocioni/mul_env/bin/python3",
    #                 "/home/jacopocioni/mulinex_ws/src/pi3hat_hw_interface/launch/set_motor_params.py",
    #                 moteus_pi3hat_path,
    #                 moteus_pi3hat_pkg_path])
    set_motor_par_script = PathJoinSubstitution([
        FindPackageShare("pi3hat_hw_interface"),
        'launch',
        'set_motor_params.py'
       ]
    )
    init_proc = ExecuteProcess(
            cmd=['python3',set_motor_par_script,moteus_pi3hat_path,moteus_pi3hat_pkg_path],
            shell=True,
            output='screen',
        )
    

    robot_description = ParameterValue(
        Command(["xacro ",moteus_pi3hat_path]),
        value_type=str
    )

    # #get controller config file
    # controller_path = get_package_share_path("pi3hat_hw_interface")
    
    # controller_path = os.path.join(controller_path,'config','test_config.yaml')
    controller_param = PathJoinSubstitution([FindPackageShare("pi3hat_hw_interface"),"config",conf_name_val])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},controller_param],
        
    )

    exec_node  = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=init_proc,
            on_exit=[control_node],
        )
    )


    return LaunchDescription(
        [
        #    moteus_pi3hat_model,
        #     control_node,
            urdf_name_arg,
            conf_name_arg,

            init_proc,
            exec_node
        ]
    )
