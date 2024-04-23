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
   
    
    use_imu_value = LaunchConfiguration("use_imu")
    main_t_value = LaunchConfiguration("main_t")
    can_t_value = LaunchConfiguration("can_t")
    rec_t_value = LaunchConfiguration("rec_t")
   
    use_imu_arg = DeclareLaunchArgument("use_imu",default_value="0")
    main_t_arg = DeclareLaunchArgument("main_t",default_value="500000")
    can_t_arg = DeclareLaunchArgument("can_t",default_value="2000")
    rec_t_arg = DeclareLaunchArgument("rec_t",default_value="2000")
    moteus_pi3hat_path = PathJoinSubstitution([
        FindPackageShare("pi3hat_hw_interface"),
        "urdf",
       "bench_test.urdf.xacro"
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
        Command(["xacro ",moteus_pi3hat_path, " use_imu:=",use_imu_value," main_t_arg:=",main_t_value," can_t_arg:=",can_t_value," rec_t_arg:=",rec_t_value]),
        value_type=str
    )

    # #get controller config file
    # controller_path = get_package_share_path("pi3hat_hw_interface")
    
    # controller_path = os.path.join(controller_path,'config','test_config.yaml')
    controller_param = PathJoinSubstitution([FindPackageShare("pi3hat_hw_interface"),"config","bench_controller.yaml"])
    
    # print(controller_path)
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
            # urdf_name_arg,
            # conf_name_arg,
            use_imu_arg,
            main_t_arg,
            can_t_arg,
            rec_t_arg,
            init_proc,
            exec_node
        ]
    )
