
import math
import asyncio
import moteus
import moteus_pi3hat
import time
import xml.etree.ElementTree as el
import os
import datetime
import yaml
import sys


general_params = {
	"max_pow":450,
	"max_cur":20,
    "flux_min_v":27.5,
}



def parse_jnt_param(jnt : el.Element):
    get_req_par = [False,False]
    jnt_name = jnt.attrib["name"]
    ret_dict = {"params" :{"name": jnt_name,"bus": 0, "id": 0, "KP": 0.0, "KD": 0.0, "KI": 0.0, "i_limit" : 0.0, "min_pos":0.0, "max_pos": 0.0, "pos_offset":0.0, "max_vel":0.0 }}
    for jnt_par in jnt.iter("param"):
        param_name = jnt_par.attrib["name"]
        param_val = jnt_par.text
        if(param_val == None):
            raise Exception(f"the joint {jnt_name} parameter {param_name} is empty")
        if param_name == "bus":
            get_req_par[0] = True
            ret_dict["params"]["bus"] = int(param_val)
        elif param_name == "id":
            get_req_par[1] = True
            ret_dict["params"]["id"] = int(param_val)
        elif param_name == "KP":
            ret_dict["params"]["KP"] = float(param_val)
        elif param_name == "KD":
            ret_dict["params"]["KD"] = float(param_val)
        elif param_name == "KI":
            ret_dict["params"]["KI"] = float(param_val)
        elif param_name == "i_limit":
            ret_dict["params"]["i_limit"] = float(param_val)
        elif param_name == "p_lim_max":
            ret_dict["params"]["p_lim_max"] = float(param_val)
        elif param_name == "p_lim_min":
            ret_dict["params"]["p_lim_min"] = float(param_val)
        elif param_name == "p_offset":
            ret_dict["params"]["p_offset"] = float(param_val)
        elif param_name == "motor_transmission":
            ret_dict["params"]["motor_transmission"] = float(param_val)
        elif param_name == "sec_enc_transmission":
            ret_dict["params"]["sec_enc_transmission"] = float(param_val)
        elif param_name == "max_vel":
            ret_dict["params"]["max_vel"] = float(param_val)
        else:
            raise Exception("Unrecognize parameter in urdf") 
   
    if not (get_req_par[0] and get_req_par[1]):
        raise Exception(f"Not provided the Mandatory joint {jnt_name} parameters")   
    
    return ret_dict

def get_moteus_lists(params : dict):
    moteus_dict = {1:[],2:[],3:[],4:[]}
    
    par_list = [[],[],[],[],[],[],[],[]]
    joints = params["robot_param"]
    for jnt in joints:
        
        jnt_id = jnt["params"]["id"]
        jnt_bus = jnt["params"]["bus"]
        jnt_kp = jnt["params"]["KP"]
        jnt_kd = jnt["params"]["KD"]
        jnt_ki = jnt["params"]["KI"]
        
        mot_trans = jnt["params"]["motor_transmission"]
        max_pos = jnt["params"]["p_lim_max"]
        min_pos = jnt["params"]["p_lim_min"]
        pos_offset = jnt["params"]["p_offset"]
        max_vel = jnt["params"]["max_vel"]
        jnt_i_limit = jnt["params"]["i_limit"]
        if jnt_id in par_list[0]:
            print("not unique id")
            raise Exception("All IDs must be unique")
        if not (jnt_bus in [1,2,3,4]):
            print("incorrect bus")
            raise Exception("The bus value are out of range")
        print(f"the motor with id-bus: {jnt_id}-{jnt_bus} has position offset {pos_offset}")
        par_list[0].append(jnt_id)
        par_list[1].append(from_jnt_to_motor_gain(jnt_kp,mot_trans))
        par_list[2].append(from_jnt_to_motor_gain(jnt_kd,mot_trans))
        par_list[3].append(from_jnt_to_motor_gain(jnt_ki,mot_trans))
        par_list[4].append(jnt_i_limit)
        if max_pos == 0.0: 
           par_list[5].append(math.nan)
        else:
           par_list[5].append(from_jnt_to_motor(max_pos + pos_offset,mot_trans))
        if min_pos == 0.0:
           par_list[6].append(math.nan)
        else:
           par_list[6].append(from_jnt_to_motor(min_pos + pos_offset,mot_trans))
        if max_vel == 0.0:
           par_list[7].append(math.nan)
        else:
           par_list[7].append(from_jnt_to_motor(max_vel,mot_trans))
        moteus_dict[jnt_bus].append(jnt_id)
    return [par_list,moteus_dict]

def from_jnt_to_motor(val, motor_trans):
    ret = (val * motor_trans )/(2*math.pi)
    if ret >= pow(2,16):
        ret = pow(2,16) - 1
    elif ret <= - pow(2,16):
        ret = pow(2,16) - 1
    return ret
	 
def from_jnt_to_motor_gain(val, motor_trans):
    ret = (2*math.pi)*(val / pow(motor_trans,2) )
    if ret >= pow(2,16):
        ret = pow(2,16) - 1
    elif ret <= - pow(2,16):
        ret = pow(2,16) - 1
    return ret
    
async def main():
    
    robot_param = {"robot_param": []}
# urdf_path = os.path.join("urdf","")
    package_path = "/home/jacopocioni/mulinex_ws/src/pi3hat_hw_interface"
    urdf_file_name = "test_int.urdf.xacro"
    urdf_path = os.path.join(package_path,"urdf",urdf_file_name) 
    # print(os.path.isfile(urdf_path))
    try:
        root = el.parse(urdf_path).getroot()
    except:
        print("error in file opening")
        return 1
    print("read param form urdf")
    for joint in root.iter("joint"):
        try:
            robot_param["robot_param"].append(parse_jnt_param(joint))
        except Exception as e:
            print(e)
            return 1
    # build moteus data
    print("get info from urdf")
    try:
        [m_pars,m_dict] = get_moteus_lists(robot_param)
    except:
    	
        return 1
    
    a = input("confirm the chosen condfiguration digiting 'y': \n")
    if a != "y":
    	return 1
    print("save data in file")
    # save the robot parameter in yaml file
    conf_file_name = "Joints_Prameter_" + datetime.datetime.today().strftime('%Y_%m_%d_%H_%M_%S')

    file = open(
        os.path.join(package_path,"mul_conf_file",conf_file_name),
        "w"
    )
    yaml.dump(robot_param,file)
    file.close()
    print("start configure motors")

    
######################################################################################################################################################################################
#                                                                              SERVOS CONFIGURATION                                                                                  #
######################################################################################################################################################################################    
    # ids_leg = [1,2,3,4,5,6,7,8]#[2,1,3,4]
    # ids_wheel = [9,10,11,12]
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = m_dict
    )

    qr = moteus.QueryResolution()
    qr.q_current = moteus.F32
    # ids = ids_leg + ids_wheel
    servos ={id: moteus.Controller(id=id, transport=transport, query_resolution = qr) for id in m_pars[0]}

    print(m_pars)
    for i in range(len(m_pars[0])):
        id = m_pars[0][i]
        print(f"execute stop id {id}")
        s = moteus.Stream(servos[id],verbose=True)
        #conf = await s.command(b'conf enumerate\n')
        max_velocity = await s.command(b'conf set servo.max_velocity ' + str(m_pars[7][i]).encode('utf-8'))
        max_power = await s.command(b'conf set servo.max_power_W ' + str(general_params["max_pow"]).encode('utf-8'))
        max_current = await s.command(b'conf set servo.max_current_A ' + str(general_params["max_cur"]).encode('utf-8'))
        kp = await s.command(b'conf set servo.pid_position.kp ' + str(m_pars[1][i]).encode('utf-8'))
        kd = await s.command(b'conf set servo.pid_position.kd ' + str(m_pars[2][i]).encode('utf-8')) 
        ki = await s.command(b'conf set servo.pid_position.ki ' + str(m_pars[3][i]).encode('utf-8'))
        ilimit = await s.command(b'conf set servo.pid_position.ilimit ' + str(m_pars[4][i]).encode('utf-8'))
        await s.command(b'conf set servopos.position_min ' + str(m_pars[6][i]).encode('utf-8'))
        await s.command(b'conf set servopos.position_max ' + str(m_pars[5][i]).encode('utf-8'))

        flux_brake_voltage = await s.command(b'conf set servo.flux_brake_min_voltage ' + str(general_params["flux_min_v"]).encode('utf-8'))
        await s.command(b'd index 0.0')
        await s.command(b'conf set servo.default_timeout_s ' + str(1).encode('utf-8'))
    
    return 0
        
######################################################################################################################################################################################
#                                                                                        RUN                                                                                         #
######################################################################################################################################################################################   
if __name__ == '__main__':
   ret = asyncio.run(main())
   sys.exit(ret)
