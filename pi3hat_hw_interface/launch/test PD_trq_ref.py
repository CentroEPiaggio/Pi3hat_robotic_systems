#! /home/jacopocioni/mul_env/bin/python3.9
import math
import asyncio
import moteus
import moteus_pi3hat
import time



async def main():
######################################################################################################################################################################################
#                                                                              SERVOS CONFIGURATION                                                                                  #
######################################################################################################################################################################################    
    # ids_leg = [1,2,3,4,5,6,7,8]#[2,1,3,4]
    # ids_wheel = [9,10,11,12]
    id = [1,2,3] #motor ids
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
               1:[1,2,3] 
        }
    )#motor id bus map dict<bus,id_list_per_bus>

    qr = moteus.QueryResolution()
    qr.q_current = moteus.F32
    ids = ids_leg + ids_wheel
    servos ={id: moteus.Controller(id=id, transport=transport, query_resolution = qr) for id in ids}

    
    
    for id in ids_leg:
        print(f"execute stop id {id}")
        s = moteus.Stream(servos[id],verbose=True)
        #conf = await s.command(b'conf enumerate\n')
        max_velocity = await s.command(b'conf set servo.max_velocity ' + str(param_dict_leg["max_vel"]).encode('utf-8'))
        max_power = await s.command(b'conf set servo.max_power_W ' + str(param_dict_leg["max_pow"]).encode('utf-8'))
        max_current = await s.command(b'conf set servo.max_current_A ' + str(param_dict_leg["max_cur"]).encode('utf-8'))
        kp = await s.command(b'conf set servo.pid_position.kp ' + str(param_dict_leg["kp"]).encode('utf-8'))
        kd = await s.command(b'conf set servo.pid_position.kd ' + str(param_dict_leg["kd"]).encode('utf-8')) 
        ilimit = await s.command(b'conf set servo.pid_position.ilimit 0')
        ki = await s.command(b'conf set servo.pid_position.ki ' + str(param_dict_leg["ki"]).encode('utf-8'))
        await s.command(b'conf set servopos.position_min ' + str(param_dict_leg["pos_min"]).encode('utf-8'))
        await s.command(b'conf set servopos.position_max ' + str(param_dict_leg["pos_max"]).encode('utf-8'))

        flux_brake_voltage = await s.command(b'conf set servo.flux_brake_min_voltage ' + str(param_dict_leg["flux_min_v"]).encode('utf-8'))
        await s.command(b'd index 0.0')
        await s.command(b'conf set servo.default_timeout_s ' + str(1).encode('utf-8'))

    for id in ids_wheel:
        print(f"execute stop id {id}")
        s = moteus.Stream(servos[id],verbose=True)
        #conf = await s.command(b'conf enumerate\n')
        max_velocity = await s.command(b'conf set servo.max_velocity ' + str(param_dict_wheel["max_vel"]).encode('utf-8'))
        max_power = await s.command(b'conf set servo.max_power_W ' + str(param_dict_wheel["max_pow"]).encode('utf-8'))
        max_current = await s.command(b'conf set servo.max_current_A ' + str(param_dict_wheel["max_cur"]).encode('utf-8'))
        kp = await s.command(b'conf set servo.pid_position.kp ' + str(param_dict_wheel["kp"]).encode('utf-8'))
        kd = await s.command(b'conf set servo.pid_position.kd ' + str(param_dict_wheel["kd"]).encode('utf-8')) 
        ilimit = await s.command(b'conf set servo.pid_position.ilimit 0')
        ki = await s.command(b'conf set servo.pid_position.ki ' + str(param_dict_wheel["ki"]).encode('utf-8'))
        await s.command(b'conf set servopos.position_min ' + str(param_dict_wheel["pos_min"]).encode('utf-8'))
        await s.command(b'conf set servopos.position_max ' + str(param_dict_wheel["pos_max"]).encode('utf-8'))

        flux_brake_voltage = await s.command(b'conf set servo.flux_brake_min_voltage ' + str(param_dict_wheel["flux_min_v"]).encode('utf-8'))
        await s.command(b'd index 0.0')
        await s.command(b'conf set servo.default_timeout_s ' + str(1).encode('utf-8'))
    
    
        
######################################################################################################################################################################################
#                                                                                        RUN                                                                                         #
######################################################################################################################################################################################   
if __name__ == '__main__':
    asyncio.run(main())
