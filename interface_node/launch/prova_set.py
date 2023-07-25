#! /home/jacopocioni/mul_env/bin/python3.9
import math
import asyncio
import moteus
import moteus_pi3hat
import time

from function_lib import *

async def main():
######################################################################################################################################################################################
#                                                                              SERVOS CONFIGURATION                                                                                  #
######################################################################################################################################################################################    
    ids = [4,1]
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1:ids          
        }
    )

    qr = moteus.QueryResolution()
    qr.q_current = moteus.F32

    servos ={id: moteus.Controller(id=id, transport=transport, query_resolution = qr) for id in ids}

      
    
    for id in ids:
        s = moteus.Stream(servos[id],verbose=True)
        #conf = await s.command(b'conf enumerate\n')
        max_velocity = await s.command(b'conf set servo.max_velocity ' + str(MActPar.MAXVEL).encode('utf-8'))
        max_power = await s.command(b'conf set servo.max_power_W ' + str(MActPar.MAXPOW).encode('utf-8'))
        max_current = await s.command(b'conf set servo.max_current_A ' + str(MActPar.MAXCUR).encode('utf-8'))
        kp = await s.command(b'conf set servo.pid_position.kp ' + str(MActPar.KP).encode('utf-8'))
        kd = await s.command(b'conf set servo.pid_position.kd ' + str(MActPar.KD).encode('utf-8')) 
        ilimit = await s.command(b'conf set servo.pid_position.ilimit 0')
        ki = await s.command(b'conf set servo.pid_position.ki ' + str(MActPar.KI).encode('utf-8'))
        flux_brake_voltage = await s.command(b'conf set servo.flux_brake_min_voltage ' + str(MActPar.FBV).encode('utf-8'))
        await s.command(b'd index 0.0')
    
        
######################################################################################################################################################################################
#                                                                                        RUN                                                                                         #
######################################################################################################################################################################################   
if __name__ == '__main__':
    asyncio.run(main())