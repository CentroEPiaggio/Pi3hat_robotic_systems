import math
import asyncio
import moteus
import moteus_pi3hat
import time
import csv

async def main():
######################################################################################################################################################################################
#                                                                              CONFIGURATION VARIABLES                                                                                  #
######################################################################################################################################################################################    
    can_bus = 1
    motor_ids = [4,3,2,1]
    sup_pos_limit = 1
    inf_pos_limit = -1
    MAXVEL = 1
    MAXPOW = 450
    MAXCUR = 40 
    KP = 3
    KD = 0.05
    KI = 0
    FBV = 27.5
    RID = 9
    Period = 5 #[s]
    Revolutions = 10
    no_resp_count = 0

    cmd_positions = []
    msr_positions = []
    times = []
    filtered_msr_pos = []

######################################################################################################################################################################################
#                                                                              SERVOS CONFIGURATION                                                                                  #
######################################################################################################################################################################################    
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            can_bus:[motor_ids]           
        },
    )

    qr = moteus.QueryResolution()
    qr._extra[moteus.Register.ENCODER_1_POSITION] = moteus.F32
    qr._extra[moteus.Register.ENCODER_1_VELOCITY] = moteus.F32

    # if(qr._extra[moteus.Register.ENCODER_0_POSITION]==moteus.F32):
    #     print("correct")
    # else:
    #     print("incorrect")


 
    controllers = { id :moteus.Controller(id=id, transport=transport, query_resolution = qr) for id in motor_ids}
    for id in motor_ids:
        print(id)
        s = moteus.Stream(controllers[id], verbose=True)
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

    command = []
    for id in motor_ids:
        command.append(controllers[id].make_stop())
    await transport.cycle(command)

    print("Revolution Starts")
    start = time.time()
    while True:        
        now = time.time()
        t = now - start
        
        
        
        pos = math.sin(2*math.pi*t/Period)
        vel = math.cos(2*math.pi*t/Period)

        
        # print(f"pos {pos} vel {vel}")
        command = []
        for id in motor_ids:
            command.append(controllers[id].make_position(
            position= pos,
            velocity= vel,
            
            query= True
        ) )
            
        result = await transport.cycle(command)
        if len(result) != 0:
            times.append(t)
            cmd_positions.append(pos)
            msr_positions.append(result[0].values[moteus.Register.POSITION])
            #filtered_msr_pos.append(result[0].values[moteus.Register.ENCODER_1_POSITION])
            
        else:
            no_resp_count = no_resp_count + 1

        if t > Period*Revolutions:
            break
    
    await transport.cycle([controllers[id].make_stop() for id in motor_ids])
    print("Revolutions Complete")
    print(len(msr_positions),len(cmd_positions),len(times))
    if len(msr_positions)== len(cmd_positions) and len(cmd_positions) == len(times) and len(times) == len(msr_positions):
        dim = len(msr_positions)
        data = open('Rev_res.csv',"w+")
        writer = csv.writer(data)
        #writer.writerow(["time","cmd_pos","msr_pos"])

        print(" the csv data has time/command_position/measured_position")

        for i in range(dim):
            positions = [times[i],cmd_positions[i],msr_positions[i],filtered_msr_pos[i]]
           
            writer.writerow(positions)
            # data.write("".join(
                  
            #       f"{times[i]}, "+
            #       f"{cmd_positions[i]}, " +
            #       f"{msr_positions[i]}\n " )) 
        data.close()
    
    print(f"the not received messages are {no_resp_count}")
    



        
   

######################################################################################################################################################################################
#                                                                                   STOP ALL MOTORS                                                                                  #
######################################################################################################################################################################################    
    await transport.cycle([controllers[id].make_stop() for id in motor_ids])

######################################################################################################################################################################################
#                                                                                        RUN                                                                                         #
######################################################################################################################################################################################   
if __name__ == '__main__':
    asyncio.run(main())