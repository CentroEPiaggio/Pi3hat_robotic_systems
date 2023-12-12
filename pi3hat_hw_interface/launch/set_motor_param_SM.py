#! /home/jacopocioni/mul_env/bin/python3.9
import math
import asyncio
import moteus
import moteus_pi3hat
import time

param_dict_hfe = {
	"kp":10.0,
	"kd":0.05,
	"max_vel":20,
	"max_pow":450,
	"max_cur":20,
	"ki":0.0,
	"flux_min_v":27.5,
    "pos_min":-9,
    "pos_max":9
}
param_dict_kfe = {
	"kp":5.0,
	"kd":0.025,
	"max_vel":200,
	"max_pow":450,
	"max_cur":80,
	"ki":0.0,
	"flux_min_v":27.5,
    "pos_min":-9,
    "pos_max":9
}


async def main():
######################################################################################################################################################################################
#                                                                              SERVOS CONFIGURATION                                                                                  #
######################################################################################################################################################################################    
    ids_hip = [1,3,5,7]#[2,1,3,4]
    ids_knee = [2,4,6,8]
    print(f"hip {ids_hip} knee {ids_knee}")
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
	    3:[1,2],
	    4:[3,4],
	    1:[5,6],
	    2:[7,8]
        }
    )

    qr = moteus.QueryResolution()
    qr.q_current = moteus.F32
    ids = ids_hip + ids_knee
    servos ={id: moteus.Controller(id=id, transport=transport, query_resolution = qr) for id in ids}

    
    
    for id in ids_hip:
        print(f"execute stop id {id}")
        s = moteus.Stream(servos[id],verbose=True)
        #conf = await s.command(b'conf enumerate\n')
        max_velocity = await s.command(b'conf set servo.max_velocity ' + str(param_dict_hfe["max_vel"]).encode('utf-8'))
        max_power = await s.command(b'conf set servo.max_power_W ' + str(param_dict_hfe["max_pow"]).encode('utf-8'))
        max_current = await s.command(b'conf set servo.max_current_A ' + str(param_dict_hfe["max_cur"]).encode('utf-8'))
        kp = await s.command(b'conf set servo.pid_position.kp ' + str(param_dict_hfe["kp"]).encode('utf-8'))
        kd = await s.command(b'conf set servo.pid_position.kd ' + str(param_dict_hfe["kd"]).encode('utf-8')) 
        ilimit = await s.command(b'conf set servo.pid_position.ilimit 0')
        ki = await s.command(b'conf set servo.pid_position.ki ' + str(param_dict_hfe["ki"]).encode('utf-8'))
        await s.command(b'conf set servopos.position_min ' + str(param_dict_hfe["pos_min"]).encode('utf-8'))
        await s.command(b'conf set servopos.position_max ' + str(param_dict_hfe["pos_max"]).encode('utf-8'))

        flux_brake_voltage = await s.command(b'conf set servo.flux_brake_min_voltage ' + str(param_dict_hfe["flux_min_v"]).encode('utf-8'))
        await s.command(b'd index 0.0')
        await s.command(b'conf set servo.default_timeout_s ' + str(1).encode('utf-8'))

    for id in ids_knee:
        print(f"execute stop id {id}")
        s = moteus.Stream(servos[id],verbose=True)
        #conf = await s.command(b'conf enumerate\n')
        max_velocity = await s.command(b'conf set servo.max_velocity ' + str(param_dict_kfe["max_vel"]).encode('utf-8'))
        max_power = await s.command(b'conf set servo.max_power_W ' + str(param_dict_kfe["max_pow"]).encode('utf-8'))
        max_current = await s.command(b'conf set servo.max_current_A ' + str(param_dict_kfe["max_cur"]).encode('utf-8'))
        kp = await s.command(b'conf set servo.pid_position.kp ' + str(param_dict_kfe["kp"]).encode('utf-8'))
        kd = await s.command(b'conf set servo.pid_position.kd ' + str(param_dict_kfe["kd"]).encode('utf-8')) 
        ilimit = await s.command(b'conf set servo.pid_position.ilimit 0')
        ki = await s.command(b'conf set servo.pid_position.ki ' + str(param_dict_kfe["ki"]).encode('utf-8'))
        await s.command(b'conf set servopos.position_min ' + str(param_dict_kfe["pos_min"]).encode('utf-8'))
        await s.command(b'conf set servopos.position_max ' + str(param_dict_kfe["pos_max"]).encode('utf-8'))

        flux_brake_voltage = await s.command(b'conf set servo.flux_brake_min_voltage ' + str(param_dict_kfe["flux_min_v"]).encode('utf-8'))
        await s.command(b'd index 0.0')
        await s.command(b'conf set servo.default_timeout_s ' + str(1).encode('utf-8'))
    
    
        
######################################################################################################################################################################################
#                                                                                        RUN                                                                                         #
######################################################################################################################################################################################   
if __name__ == '__main__':
    asyncio.run(main())
