from launch import LaunchDescription
from launch_ros.actions import Node


import subprocess



def generate_launch_description():

    subprocess.run(["sudo",
                    "/home/jacopocioni/mul_env/bin/python3",
                    "/home/jacopocioni/mulinex_ws/src/interface_node/launch/prova_set.py"])
    print("i' have executed the configuration process")
    node_inter =  Node(
            package='interface_node',
            executable='interface_node_v1',
            name='prova'
        )
    return LaunchDescription(
        [
            node_inter
        ]
    )