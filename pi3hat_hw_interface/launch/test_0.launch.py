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
   
    urdf_name = LaunchConfiguration("urdf_file",default="8DOF_mul_off.urdf.xacro")
    conf_name = LaunchConfiguration("conf_file",default="test_config.yaml")

    urdf_name_arg = DeclareLaunchArgument("urdf_name",default_value="8DOF_mul_off.urdf.xacro")
    conf_name_arg = DeclareLaunchArgument("conf_name",default_value="test_config.yaml")
    print(10)
    moteus_pi3hat_path = PathJoinSubstitution([
        FindPackageShare("pi3hat_hw_interface"),
        "urdf","8DOF_mul_off.urdf.xacro"
       ]
    )
    print(ParameterFile(moteus_pi3hat_path,allow_substs=True))
    moteus_pi3hat_path = os.path.join(moteus_pi3hat_pkg_path,"urdf/8DOF_mul_off.urdf.xacro") 

    # print(moteus_pi3hat_path)
    res = subprocess.run(["sudo",
                    "/home/jacopocioni/mul_env/bin/python3",
                    "/home/jacopocioni/mulinex_ws/src/pi3hat_hw_interface/launch/set_motor_params.py",
                    moteus_pi3hat_path,
                    moteus_pi3hat_pkg_path
                    ])
    
    assert res.returncode == 0 , "raised error in configuration process"
    # print("i' have executed the configuration process")
    print(10)
    # moteus_pi3hat_path = get_package_share_path("pi3hat_hw_interface")
    # moteus_pi3hat_path = os.path.join(moteus_pi3hat_path,"urdf/test_int.urdf.xacro") 
    moteus_pi3hat_model = DeclareLaunchArgument(
        name="moteus_pi3hat_urdf",
        default_value=str(moteus_pi3hat_path)
    )
    robot_description = ParameterValue(
        Command(["xacro ",LaunchConfiguration("moteus_pi3hat_urdf")]),
        value_type=str
    )

    #get controller config file
    controller_path = get_package_share_path("pi3hat_hw_interface")
    
    controller_path = os.path.join(controller_path,'config','test_config.yaml')
    # controller_param = PathJoinSubstitution( [FindPackageShare("pi3hat_hw_interface"),"config",conf_name])
    
    print(controller_path)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},controller_path],
        
    )

    # load_cont_lh = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'wb_forward_position_controller'],
    #     output='screen'
    # )

    # solo12_broadcast = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'solo12_broadcast'],
    #     output='screen'
    # )
#    joy_event_node = Node(
#	package="joy",
#	executable="joy_node",
#	output="screen")

    return LaunchDescription(
        [
           moteus_pi3hat_model,
            control_node,
            urdf_name_arg,
            conf_name_arg
#            joy_event_node
        ]
    )
