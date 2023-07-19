import math
import asyncio
import moteus
import moteus_pi3hat
import time


async def main():
######################################################################################################################################################################################
#                                                                              CONFIGURATION VARIABLES                                                                                  #
######################################################################################################################################################################################    
    can_bus = 1
    motor_id = 1
    sup_pos_limit = 1
    inf_pos_limit = -1
    MAXVEL = 1
    MAXPOW = 450
    MAXCUR = 40 
    KP = 4
    KD = 0.01
    KI = 0
    FBV = 27.5
    RID = 9
    Period = 10 #[s]
    Revolutions = 2
    no_resp_count = 0

    cmd_positions = []
    msr_positions = []
    times = []


######################################################################################################################################################################################
#                                                                              SERVOS CONFIGURATION                                                                                  #
######################################################################################################################################################################################    
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            can_bus:[motor_id]           
        },
    )

    qr = moteus.QueryResolution()
    qr.q_current = moteus.F32

 
    controller = moteus.Controller(id=motor_id, transport=transport, query_resolution = qr)
    s = moteus.Stream(controller, verbose=True)
    max_velocity = await s.command(b'conf set servo.max_velocity ' + str( MAXVEL).encode('utf-8'))
    max_power = await s.command(b'conf set servo.max_power_W ' + str( MAXPOW).encode('utf-8'))
    max_current = await s.command(b'conf set servo.max_current_A ' + str( MAXCUR).encode('utf-8'))
    kp = await s.command(b'conf set servo.pid_position.kp ' + str( KP).encode('utf-8'))
    kd = await s.command(b'conf set servo.pid_position.kd ' + str( KD).encode('utf-8')) 
    ilimit = await s.command(b'conf set servo.pid_position.ilimit 0')
    ki = await s.command(b'conf set servo.pid_position.ki ' + str( KI).encode('utf-8'))
    flux_brake_voltage = await s.command(b'conf set servo.flux_brake_min_voltage ' + str( FBV).encode('utf-8'))
    pos_inf_lim = await s.command(b'conf set servopos.position_min ' + str( inf_pos_limit).encode('utf-8'))
    pos_sup_lim = await s.command(b'conf set servopos.position_max ' + str( sup_pos_limit).encode('utf-8'))

    await s.command(b'd index 0.0')
    print("passo")
    for i in range(10):
        await transport.cycle([controller.make_stop()])
        print("stop")
if __name__ == '__main__':
    asyncio.run(main())