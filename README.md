# Pi3hat_robotic_systems
ROS2_control hardware interface to hook up the pi3hat/moteus base robotic system 

Note 1.
when the pi3hat system is configuring, i.e when the Kp,Kd,pos_limit, etc. are set, the user must not use parameter that exceed the representativity on 16 bit, otherwise this operation will write in memory different memory 
zone causing unexpected, dangerous and even mortal behavior.
