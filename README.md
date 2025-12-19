# Pi3hat_robotic_systems
ROS2_control hardware interface to hook up the pi3hat/moteus base robotic system. it allow the user to control the MJBots Moteus driver collocated in a general porpouse robotic actuator, eventually adding a second encoder.
The ros2 packages are developed to be run on a raspberry connected to the MJBots Pi3Hat system, it performs as a fd-can master and IMU sensor manager.

<!-- Note 1.
when the pi3hat system is configuring, i.e when the Kp,Kd,pos_limit, etc. are set, the user must not use parameter that exceed the representativity on 16 bit, otherwise this operation will write in memory different memory 
zone causing unexpected, dangerous and even mortal behavior. -->

## Prerequisite 

Raspberry Pi 4 running ubuntu 22.04 and ROS Humble [(tested on this image)](https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/tag/22.04.3_v5.15.98-rt62-raspi_ros2_humble)

pi3hat r 4.4 or newer



## Raspberry Setup 

<ol>

<li> 

install the cpp compiler package 

``` sudo apt-get install build-essential  ```

</li>

<li> 

install the colcon builder packagers

``` sudo apt install python3-colcon-common-extensions  ```

</li>

<li> 

allows the code to access to the bmc_host library, needed by 

``` sudo ln /usr/lib/aarch64-linux-gnu/libbcm_host.so /usr/lib/libbcm_host.so.0 ```

```  sudo apt-get install libraspberrypi-dev```
</li>

<li> 

install ros2 control framework and xacro 

```  sudo apt-get install ros-humble-ros2-control```

```  sudo apt-get install ros-humble-ros2-controllers```

``` sudo apt install ros-humble-xacro  ```
</li>

<li> 
    install pip3 and the moteus libraries:

    
 ```  sudo apt install python3-pip  ```

  ```   sudo pip3 install moteus==0.3.67  ```

 ```   sudo pip3 install moteus_pi3hat  ```


</li> 


<li>
    Optional: install the Network-Manager to use the Raspberry PI4 as Access Point

```   sudo apt install network-manager  ```
</li>

</ol>


## ROS2 Packages
<ol>
<li>
    moteus_pi3hat: is the ros2 package version of the original code to operate on the pi3hat using moteus driver
</li>

<li>
    pi3hat_hw_interface: contains the hardware interface implementation 
</li>

<li>
    pi3hat_moteus_int_msgs: contains the interfaces to use all the controllers available in this repository.
    <ol>
    <li>
        JointCommand: is a message used by the joint_controller, it is a sort of sensor_msgs/JointStates message contains also the scales associate with the proportional and the derivative gains.
    </li>
    <li>
        JointStates: is a message used by the state_broadcaster, it is a sort of sensor_msgs/JointStates message contains also the temperature, the current and eventually the second encoder position and velocity.
    </li>
    <li>
        PacketPass: is a message used by the state_broadcaster. it contains the information about the performance of the low level fd can comunication. it provide the state of the whole comunication composed by the communication cycle period in second and the validity loss, i.e. the not completed comminication cycle amount per hundred iteration. Then it provide also the each motor packet loss amount per hundred iteration when the communication cycle is completed.
    </li>
    <li>
        OmniMulinexCommand: message used by the omni_wheels_controller. it provide the floating base 2D twist with the heigth rate. NB used only for demos.
    </li>
    </ol>
     
</li>
<li>
    pi3hat_base_controllers: contains two base controllers. The joints_controller can be use to control each joint indipendently enforsing the drivers low level PID control using the JointCommand messages. The state_broadcaster controller can be use to monitor the joints states and the low level communication state using the JointStates and the PacketPass Messages 

</li>
<li>
    pi3hat_omni_controller: contains a controller to perform the omni_quad Demos.
</li>
</ol>

## Interface configuration file
The [ROS2 Control](https://control.ros.org/master/index.html) framework must be known to understand this part.
The interface allows the user to manage the moteus r-series, the distributors and the integrated inertia unit on the pi3hat.

The Interfaces parameters can be divided into four groups:
<ul>
<li>
    pi3hat parameters, used to regulate the low level communication and to use the integrated IMU sensor.
</li>
<li>
    control actuator parameters, used to set up the low level PID and the communication daisy chain.
</li>
<li>
    measure resolution actuator parameters, used to define the actuator state interface that the interface open.
</li>
<li>
    measure resolution distributor parameters, uused to define the distributor state interface that the interface open.
</li>
</ul>
The Resolution can be set to 64, 32,16 and 8 bit to get the data with a certain representation or 0 to disable the measure getting, for more information about the moteus and distributor available measure check [Moteus Dirver](https://github.com/mjbots/moteus/blob/main/docs/reference.md) and [Power Distributor](https://github.com/mjbots/power_dist/blob/main/docs/reference.md)
These parameter must be set into the xacro contain into pi3hat_ha_interface's config folder.
An example of the URDF file is:


``` <?xml version="1.0" ?>
<robot name="softleg_cart"  xmlns:xacro="http://ros.org/wiki/xacro">
    <ros2_control name="MoteusPi3Hat_Interface" type="system">
        <hardware>
            <plugin>pi3hat_hw_interface/MoteusPi3Hat_Interface</plugin>
            <param name="timeout_ns">0</param>
            <param name="min_tx_wait_ns">20000</param>
            <param name="rx_baseline_wait_ns">200000</param>
            <param name="rx_extra_wait_ns">20000</param>
            <param name="CPU_affinity">1</param>

            <param name="request_attitude">1</param> 

            <param name="mounting_deg_roll">0</param> 
            <param name="mounting_deg_pitch">0</param> 
            <param name="mounting_deg_yaw">0</param> 

            <param name="attitude_hz">400</param> 
        </hardware>

        <joint name="HIP"> 
            <param name="id">4</param>
            <param name="bus">1</param>
            <param name="second_encoder_source">1</param>
            <param name="type">motor</param>
            
            <param name="KP">100.0</param> 
            <param name="KD">1.0</param> 
            <param name="KI">0.0</param> 
            <param name="ilimit">0.0</param>
            <param name="iratelimit">0.0</param>
            <param name="max_position_slip">0.0</param>
            <param name="max_velocity_slip">0.0</param>
            <param name="enable_motor_temperature">0</param>
            <param name="max_pos_limit">0.0</param> 
            <param name="min_pos_limit">0.0</param>
            <param name="max_velocity">0.0</param>
            <param name="max_effort">20000.0</param>
            <param name="actuator_trasmission">9.0</param>
            <param name="position_offset">-0.1059179</param>
            <param name="second_encoder_trasmission">3.23076923</param> 

            <param name="position_res">64</param> 
            <param name="velocity_res">64</param> 
            <param name="torque_res">64</param> 
            <param name="q_current_res">8</param>
            <param name="d_current_res">0</param>
            <param name="abs_position_res">0</param>
            <param name="power_res">0</param>
            <param name="motor_temperature_res">0</param>
            <param name="voltage_res">0</param> 
            <param name="temperature_res">8</param>
            <param name="position_error_res">64</param>
            <param name="velocity_error_res">64</param>
            <param name="torque_error_res">64</param>
            <param name="second_encoder_position_res">64</param>
            <param name="second_encoder_velocity_res">64</param> 
        </joint>

        <joint name="Leg_Distributor"> 
            <param name="id">32</param>
            <param name="bus">5</param>
            <param name="type">power_dist</param>
            <param name="voltage">64</param> 
            <param name="current">64</param> 
            <param name="temperature">64</param> 
            <param name="energy">0</param>
            <param name="state">0</param>
            <param name="switch_state">0</param>
            <param name="lock_time">0</param>
            <param name="boot_time">0</param>


        </joint>

    </ros2_control>
</robot>
 

```

<ol>
    <li>
    Communication parameter:
    <ul>
        <li>
            timeout_ns: at least the amount of time use by to complete the communication cycle     
        </li>
        <li>
            min_tx_wait_ns:  at least the amount of time use by to complete the communication cycle after the pi3hat transmission is end.
        </li>
        <li>
            rx_baseline_wait_ns: at least the amount of time wait to recive all the packet.
        </li>
        <li>
            rx_extra_wait_ns: is an extra timeout, it is add to the base one if a verified(CRC) message has beed received.
        </li>
        <li>
            CPU_affinity: if greater than zero associate the communication to a ARM micro Core. PS the raspberry core are four.
        </li>
    </ul>
    </li>
     <li>
    IMU parameter:
    <ul>
        <li>
            request_attitude: if set to zero the pi3hat do not collect the IMU data, is available just the AHRS filter embedded with the pi3hat.     
        </li>
        <li>
            mounting_deg_[roll/pitch/yaw]: orientation offset of the pi3hat frame respect the a desired floating base frame
        </li>
         <li>
            attitude_hz: publish frequency of the embedded AHRS fiter.
        </li>
    </ul>
    </li>
      <li>
    Joints parameter:
    <ul>
        <li>
            KP,KD,KI,i_limit: parameter for the low level PID controller
        </li>
        <li>
            bus,id: parameter to identify the pi3hat can bus port and the driver id number
        </li>
        <li>
            motor_transmission: value of the transmission rateo from the motor to the actuator joint, all the motor quantities provided by the interfaces take into account this value
        </li>
          <li>
            sec_enc_transmission: value of the transmission between actiator joint and the second encoder, the second encoder quantities take into account  this value. if set to zero the the interface do not provide the second econder measures.
        </li>
        <li>
            p_lim_max,p_lim_min: joint level saturarion position of the actuator in radians, if set to zero  the limit is disabled.
        </li>
        <li>
            p_offset: joint level offset position of the actuator in radians, if set to zero  the limit is disabled.
        </li>
        <li>
            max_vel: joint level velocity saturarion.
        </li>
        <li>
            max_torque: joint level torque saturation.
        </li>
        <li>
            max_position_slip: When finite, this enforces a limit on the difference between the control position and the current measured position measured in revolutions. It can be used to prevent "catching up" when the controller is used in velocity mode. if set to 0.0 is disabled.
        </li>
        <li>
            max_velocity_slip: When finite, this enforces a limit on the difference between the control velocity and the current measured velocity, measured i Hz. It can be used to ensure that acceleration limits are obeyed in velocity mode when external torques exceed the maximum. If used, typically servo.max_position_slip must be relatively small to avoid instability. if set to 0.0 is disabled.
        </li>
    </ul>
    </li>
</ol>

## Joints_Controller 
### Parameters
<ul>
    <li>joints: the list of all joint</li>
    <li>init_pos: the list of all joints start position, if not provided it will be zero at all joints</li>
    
</ul>

### Topic 
joint_controller/command: Message type pi3hat_moteus_int_msgs/JointCommand. 
<ul>
    <li>Joints Name, list of joints name</li>
    <li>Positions reference, if NaN the PI control is disabled</li>
    <li>Velocities reference, if NaN the D control is disabled</li>
    <li>Torques reference, if NaN the motor will fault</li>
    <li>kp_gain_scale, base propotional gain scale factor </li>
    <li>kd_gain_scale, base derivative gain scale factor</li>
</ul>

## State_Broadcaster
### Parameters
<ul>
    <li>joints: the list of all joint</li>
    <li>second_encoders: list of joint equipped with second encoder</li>
    <li>performance_index: If true will be open also the low level communication performance indexes</li>
    
</ul>

### Topic

state_broadcaster/joints_state: Message type pi3hat_moteus_int_msgs/JointStates. 
<ul>
    <li>Joints Name, list of joints name</li>
    <li>Positions measured</li>
    <li>Velocities measured</li>
    <li>Torques measured</li>
    <li>Current measured </li>
    <li>Driver Temperature measured</li>
    <li>Second Encoder Position, if needed</li>
    <li>Second Encoder Velocity, if needed </li>
</ul>

state_broadcaster/performance_indexes: Message type pi3hat_moteus_int_msgs/PacketPass. 
<ul>
    <li>Pi3Hat Cycle time </li>
    <li>Cycle Validation, if one then the system has lost a cycle</li>
    <li>Joints name list/li>
    <li>Joints Packet loss, if one then the low level communication has lost a motor package</li>

</ul>

## Launch Interface

Set up the robot terminal:

- go into the mulinex_ws folder and make the global and local ros2 source
 ``` cd mulinex_ws ```

``` source /opt/ros/humble/setup.bash ```


``` source install/local_setup.bash ```

If a communication with opc are needed also the ROS_DOMAIN_ID must be set, the id value must be the same for both robot and OPC.

``` export ROS_DOMAIN_ID=<ID> ```

To launch the interface use the command:


``` ros2 launch pi3hat_hw_interface start_MJBots_Pi3Hat_hw_Interface.launch.py urdf_file:=<filename> conf_file:=<filename>  ```

the configuration and urdf file must be contained respectively in  pi3hat_hw_interface/config and  pi3hat_hw_interface/urdf.  
## References
[Moteus Dirver](https://github.com/mjbots/moteus/blob/main/docs/reference.md)


[Pi3hat Board](https://github.com/mjbots/pi3hat)
