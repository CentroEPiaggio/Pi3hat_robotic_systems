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
    motor_id = 12
    sup_pos_limit = 1
    inf_pos_limit = -1
    MAXVEL = 1
    MAXPOW = 450
    MAXCUR = 40 
    KP = 16
    KD = 0.08
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


    await transport.cycle([controller.make_stop()])

    print("Revolution Starts")
    start = time.time()
    while True:        
        now = time.time()
        t = now - start
        
        
        
        pos = math.sin(2*math.pi*t/Period)
        vel = math.cos(2*math.pi*t/Period)

        

        command = controller.make_position(
            position= pos,
            velocity= vel,
            feedforward_torque= 0.0,
            query= True
        )

        result = await transport.cycle([command])
        if len(result) != 0:
            times.append(t)
            cmd_positions.append(pos)
            msr_positions.append(result[0].values[moteus.Register.POSITION])
        else:
            no_resp_count = no_resp_count + 1

        if t > Period*Revolutions:
            break
    
    await transport.cycle([controller.make_stop()])
    print("Revolutions Complete")
    if len(msr_positions)!= len(cmd_positions) and len(cmd_positions) != len(times) and len(times) != len(msr_positions):
        dim = len(msr_positions)
        data = open('Revolution_Results',"w")

        print(" the csv data has time/command_position/measured_position")

        for i in range(dim):
            data.write("".join(
                  
                  f"{times[i]}, "+
                  f"{cmd_positions[i]}, " +
                  f"{msr_positions[i]}\n " )) 
    
    print(f"the not received messages are {no_resp_count}")
    



        
   

######################################################################################################################################################################################
#                                                                                   STOP ALL MOTORS                                                                                  #
######################################################################################################################################################################################    
    await transport.cycle([controller.make_stop()])

######################################################################################################################################################################################
#                                                                                        RUN                                                                                         #
######################################################################################################################################################################################   
if __name__ == '__main__':
    asyncio.run(main())