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

  ```   sudo pip3 moteus  ```

 ```   sudo pip3 moteus_pi3hat  ```


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

The Interfaces parameters can be divided into two groups:
<ul>
<li>
    pi3hat parameters, used to regulate the low level communication and to use the integrated IMU sensor.
</li>
<li>
    joints parameters, used to set up the low level PID and the communication three.
</li>
</ul>

These parameter must be set into the xacro contain into pi3hat_ha_interface's config folder.
An example of the URDF file is:


``` <?xml version="1.0" ?>
<robot name="mulinex"  xmlns:xacro="http://ros.org/wiki/xacro">
    <ros2_control name="MoteusPi3Hat_Interface" type="system">
        <hardware>
            <plugin>pi3hat_hw_interface/MoteusPi3Hat_Interface</plugin>
            <param name="main_timeout">400000</param>
            <param name="can_timeout">20000</param>
            <param name="rcv_timeout">20000</param>
            
            <param name="attitude">1</param> 

            <param name="b2imu_pos_x">0</param> 
            <param name="b2imu_pos_y">0</param> 
            <param name="b2imu_pos_z">0</param> 

            <param name="b2imu_roll">0</param> 
            <param name="b2imu_pitch">0</param> 
            <param name="b2imu_yaw">90.0</param>
            <param name="acc_correction">0</param> 
        </hardware>

        <joint name="RF_HFE"> 
            <param name="id">3</param>
            <param name="bus">4</param>
            <param name="motor_transmission">9.0</param>
            <param name="sec_enc_transmission">0.0</param> 
            <param name="KP">0.0</param> 
            <param name="KD">0.0</param> 
            <param name="KI">0.0</param> 
            <param name="i_limit">0.0</param>
            <param name="p_lim_max">0.0</param> 
            <param name="p_lim_min">0.0</param> 
            <param name="p_offset">0.0</param>
            <param name="max_vel">10.0</param>
            <param name="max_torque">5.0</param>
        </joint>

    </ros2_control>
</robot> 

```

<ol>
    <li>
    Communication parameter:
    <ul>
        <li>
            main_timeout: is the base time waited by the pi3hat recive the joint response        
        </li>
        <li>
            can_timeout: is an extra timeout, it is add to the base one if a can message has been recieved.
        </li>
        <li>
            rcv_timeout: is an extra timeout, it is add to the base one if a verified(CRC) message has beed received.
        </li>
    </ul>
    </li>
     <li>
    IMU parameter:
    <ul>
        <li>
            attitude: if set to zero the pi3hat do not collect the IMU data.     
        </li>
        <li>
            acc_correction: if set to zero it collect the raw data from IMU, else it collect the data corrected by a VHS, removing the gravity acceleration and providing the pi3hat orientation.
        </li>
        <li>
            b2imu_[roll/pitch/yaw]: orientation offset of the pi3hat frame respect the a desired floating base frame
        </li>
          <li>
            b2imu_pos_[x/y/z]: position offset of the pi3hat frame respect the a desired floating base frame
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
            p_lim_max,p_lim_min: joint level saturarion position of the actuator in radians.
        </li>
        <li>
            p_offset: joint level offset position of the actuator in radians.
        </li>
        <li>
            max_vel: joint level velocity saturarion.
        </li>
        <li>
            max_torque: joint level torque saturation.
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
joint_controller/command: Message type pi3hat_moteus_int_msgs/JointCommand. 
<ul>
    <li>Joints Name, list of joints name</li>
    <li>Positions reference, if NaN the PI control is disabled</li>
    <li>Velocities reference, if NaN the D control is disabled</li>
    <li>Torques reference, if NaN the motor will fault</li>
    <li>kp_gain_scale, base propotional gain scale factor </li>
    <li>kd_gain_scale, base derivative gain scale factor</li>
</ul>

## Launch Interface

``` ros2 launch pi3hat_hw_interface start_MJBots_Pi3Hat_hw_Interface.launch.py urdf_file:=<filename> conf_file:=<filename>  ```

the configuration and urdf file must be contained respectively in  pi3hat_hw_interface/config and  pi3hat_hw_interface/urdf.  
## References
[Moteus Dirver](https://github.com/mjbots/moteus/blob/main/docs/reference.md)


[Pi3hat Board](https://github.com/mjbots/pi3hat)
