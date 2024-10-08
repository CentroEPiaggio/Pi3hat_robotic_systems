import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

def generate_launch_description():
    
    interface_launch  = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_path('pi3hat_hw_interface'), 'launch', 'start_MJBots_Pi3Hat_hw_Interface.launch.py')
                ),
		launch_arguments={
                'urdf_file': 'omni_car.urdf.xacro',
                'conf_file': 'omni_car.yaml',
            }.items()
            )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_path('sllidar_ros2'), 'launch', 'sllidar_a2m8_launch.py')
                ),
		launch_arguments={
		'angle_compensate': 'true'
		}.items()
        )
 
    return LaunchDescription([
        interface_launch,
        lidar_launch
    ])
