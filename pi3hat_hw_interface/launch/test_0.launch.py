import os
 
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command,LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import subprocess
def generate_launch_description():


    #get solo12 urdf with system HW interface 

    res = subprocess.run(["sudo",
                    "python3",
                    "/home/mulsbc/mulinex_ws/src/pi3hat_hw_interface/launch/set_motor_params.py"])
    
    assert res.returncode == 0 , "raised error in configuration process"
    # print("i' have executed the configuration process")

    moteus_pi3hat_path = get_package_share_path("pi3hat_hw_interface")
    moteus_pi3hat_path = os.path.join(moteus_pi3hat_path,"urdf/test_int.urdf.xacro") 
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
    controller_param = PathJoinSubstitution(controller_path)
    
    print(controller_path)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},controller_param],
        
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
            control_node
#            joy_event_node
        ]
    )
