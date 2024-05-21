#ifndef MOTOR_MANAGER_HPP
#define MOTOR_MANAGER_HPP
#include "moteus_pi3hat/moteus_protocol.h"
#include "moteus_pi3hat/realtime.h"
#include "moteus_pi3hat/pi3hat_moteus_interface.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include <string>
#include <cstdint>
#include <vector>
#include <functional>
#include <tuple>
// #define NULL __null

#define MAX_COUNT 100
#define MIN_STT_INT 7

using namespace mjbots;
using namespace std;
using interface_tpl = std::tuple<std::string,const char*,double*>;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using MoteusInterface = moteus::Pi3HatMoteusInterface;
using Command = moteus::Pi3HatMoteusInterface::ServoCommand;
using Reply = moteus::Pi3HatMoteusInterface::ServoReply;
using Get_Function = std::function<moteus::QueryResultV2 ( std::vector<Reply>& replies, int bus, int id, int opt,int& err,int provided_msg)>;
using Policy_Function = std::function<void( bool msg_valid, bool msg_coplete, Command* cmd_d)>;
using Bind_Pol_Function = std::function<void(bool msg_valid, bool msg_coplete)>;
namespace hardware_interface
{
    constexpr char HW_IF_TEMPERATURE[] = "temperature";
    constexpr char HW_IF_KP_SCALE[] = "kp_scale_value";
    constexpr char HW_IF_KD_SCALE[] = "kd_scale_value";
    constexpr char HW_IF_VALIDITY_LOSS[] = "validity_loss";
    constexpr char HW_IF_PACKAGE_LOSS[] = "package_loss";
    constexpr char HW_IF_CYCLE_DUR[] = "cycle_duration";
    constexpr char HW_IF_W2R_DUR[] = "write2read_duration";
    constexpr char HW_IF_CURRENT[] = "current";
    constexpr char HW_IF_ELECT_POWER[] = "electrical_power";


}
namespace pi3hat_hw_interface
{
    
    namespace motor_manager
    {
        // it contains all the data needed to describe the single motor state
        struct motor_info
        {
            double m_position = 0.0;
            double m_velocity = 0.0;
            double m_torque = 0.0;
            double m_temperature = 0.0;
            uint8_t fault = 0;
            //double input_current;
            double enc_position = 0.0;
            double enc_velocity = 0.0;
        };

        struct motor_mem
        {
            double msr_pos = 0.0;
            double msr_vel = 0.0;
            double msr_trq = 0.0;
            double msr_tmp = 0.0;
            double msr_enc_pos = 0.0;
            double msr_enc_vel = 0.0;
            double cmd_pos = 0.0;
            double cmd_vel = 0.0;
            double cmd_trq = 0.0;
            double kp_scale = 0.0;
            double kd_scale = 0.0;
            Command* cmd_data = NULL;
            std::vector<Reply>* replies = NULL;
            bool operator=(motor_mem a)
            {
                return(
                    msr_pos == a.msr_pos &&
                    msr_vel == a.msr_vel &&
                    msr_trq == a.msr_trq &&
                    msr_tmp == a.msr_tmp &&
                    msr_enc_pos == a.msr_enc_pos &&
                    msr_enc_vel == a.msr_enc_vel &&
                    cmd_pos == a.cmd_pos &&
                    cmd_vel == a.cmd_vel &&
                    cmd_trq == a.cmd_trq &&
                    kp_scale == a.kp_scale &&
                    kd_scale == a.kd_scale
                    );
            };

        };




        class Motor_Manager
        {
            public:
                Motor_Manager(
                    std::string name = "std",
                    Command * cmd_data = nullptr,
                    std::vector<Reply>* replies = nullptr, 
                    double motor_trans = 0.0,
                    double second_trans = 0.0, 
                    uint8_t id =0, 
                    uint8_t bus = 0, 
                    Policy_Function pl_fun = nullptr,
                    Get_Function gt_fun = nullptr):
                id_(id),
                bus_(bus),
                motor_trans_(motor_trans),
                sec_enc_trans_(second_trans),
                name_(name)
                {
                    pol_callback_ = std::bind(pl_fun,_1,_2,this->cmd_data_);
                    get_callback_ = gt_fun;
                    cmd_data_ = cmd_data;
                    replies_ = replies;
                    packet_loss_ = 0;
                    inter_type_cmd_ = {
                            hardware_interface::HW_IF_POSITION,
                            hardware_interface::HW_IF_VELOCITY,
                            hardware_interface::HW_IF_EFFORT,
                            hardware_interface::HW_IF_KP_SCALE,
                            hardware_interface::HW_IF_KD_SCALE
                        };
                    inter_type_stt_ = {};
                    // RCLCPP_INFO(rclcpp::get_logger("DIO"),"pass sec_trans %f",second_trans);

                    //std::function<void(bool,bool,Command&)> a = bind(&pl_fun,this, _1,_2,_3);
                };
                ~Motor_Manager()
                {};

                void set_all(
                    std::string name,
                    Command * cmd_data,
                    std::vector<Reply>* replies, 
                    double motor_trans,
                    double second_trans, 
                    uint8_t id, 
                    uint8_t bus, 
                    double p_max,
                    double p_min,
                    double p_offset,
                    double max_eff,
                    Policy_Function pl_fun,
                    Get_Function gt_fun)
                    {
                        name_ = name;
                        cmd_data_ = cmd_data;
                        replies_ = replies;
                        motor_trans_ = motor_trans;
                        sec_enc_trans_ = second_trans;
                        id_ = id;
                        bus_ = bus;
                        p_min_ = p_min;
                        p_max_ = p_max;
                        p_offset_ = p_offset;
                        if(motor_trans_ != 0.0)
                        {
                            max_trq_ = max_eff/motor_trans_;
                            RCLCPP_INFO(rclcpp::get_logger("Moteus_Pi3hat_HW_Interface"),"the max effort for motor %s is %f",name_.c_str(),max_trq_);
                        }
                        else 
                        {   
                            RCLCPP_ERROR(rclcpp::get_logger("Moteus_Pi3hat_HW_Interface"),"motor transmission can not be zero, max effort is set to zero");
                            max_trq_ = 0.0;
                        }
                        get_callback_ = gt_fun;
                        pol_callback_ = std::bind(pl_fun,_1,_2,this->cmd_data_);
                         if(second_trans == 0.0)
                        {
                            
                            inter_type_stt_ = {
                                hardware_interface::HW_IF_POSITION,
                                hardware_interface::HW_IF_VELOCITY,
                                hardware_interface::HW_IF_EFFORT,
                                hardware_interface::HW_IF_TEMPERATURE,
                                hardware_interface::HW_IF_CURRENT,
                                hardware_interface::HW_IF_PACKAGE_LOSS,
                                hardware_interface::HW_IF_ELECT_POWER
                            
                            };
                        }
                        else
                        {
                            inter_type_stt_ = {
                                hardware_interface::HW_IF_POSITION,
                                hardware_interface::HW_IF_VELOCITY,
                                hardware_interface::HW_IF_EFFORT,
                                hardware_interface::HW_IF_TEMPERATURE,
                                hardware_interface::HW_IF_CURRENT,
                                hardware_interface::HW_IF_PACKAGE_LOSS,
                                hardware_interface::HW_IF_ELECT_POWER,
                                hardware_interface::HW_IF_POSITION,
                                hardware_interface::HW_IF_VELOCITY
                            };
                        }
                        cmd_pos_ = p_offset_;
                    };
                // set and get the current command resolution
                void set_command_resolution(moteus::PositionResolution res);
                moteus::QueryCommandV2 get_qry_res();

                // set and get the current query resolution
                void set_query_resolution(moteus::QueryCommandV2 res);
                moteus::PositionResolution get_cmd_res();
                
                // get the command location variable
                double* get_cmd_interface(std::string type);

                void set_msg_valid(bool val)
                {
                    //RCLCPP_WARN(rclcpp::get_logger("SET_VALID"),"valid is set %d",val);
                    msg_valid_ = val;
                };

                
                double get_transmission(){
                    return motor_trans_;
                };

                double get_sec_transmission()
                {
                    return sec_enc_trans_;
                };
                // get the state location variable
                double* get_stt_interface(std::string type, bool sensor);

                std::vector<std::string> get_command_type(){
                    return inter_type_cmd_;
                };

                std::vector<std::string> get_state_type()
                {
                    return inter_type_stt_;
                };

                // set the command position into the data structure
                void make_position();

                // set the command stop to all motors
                void make_stop();

                void drop_torque();

                void set_cmd(double val, int i)
                {
                    if(i==0)
                        cmd_pos_ = val;
                    else if(i==1)
                        cmd_vel_ = val;
                    else if(i == 2)
                        cmd_trq_ = val;
                    else if(i == 3)
                        cmd_kp_scale_ = val;
                    else if(i == 4)
                        cmd_kd_scale_ = val;
                    else
                        throw std::logic_error("Pass index of command pos with wrong value");
                };
                 double get_cmd( int i)
                {
                    if(i==0)
                        return cmd_pos_;
                    else if(i==1)
                        return cmd_vel_;
                    else if(i == 2)
                        return cmd_trq_;
                    else if(i == 3)
                        return cmd_kp_scale_;
                    else if(i == 4)
                        return cmd_kd_scale_;
                    else
                        throw std::logic_error("Pass index of command pos with wrong value");
                };  

                void set_stt(double val, int i)
                {
                    if(i==0)
                        msr_pos_ = val;
                    else if(i==1)
                        msr_vel_ = val;
                    else if(i == 2)
                        msr_trq_ = val;
                    else if(i == 3)
                        msr_tmp_ = val;
                    else if(i == 4)
                        msr_enc_pos_ = val;
                    else if(i == 5)
                        msr_enc_vel_ = val;
                    else
                        throw std::logic_error("Pass index of command pos with wrong value");
                };
                 double get_stt( int i)
                {
                    if(i==0)
                        return msr_pos_;
                    else if(i==1)
                        return msr_vel_;
                    else if(i == 2)
                        return msr_trq_;
                    else if(i == 3)
                        return msr_tmp_;
                    else if(i == 4)
                        return msr_enc_pos_;
                    else if(i == 5)
                        return msr_enc_vel_;
                    else
                        throw std::logic_error("Pass index of command pos with wrong value");
                };


                // get the readed value into state_vars and set flag to decide 
                int get_motor_state(int prov_msg);

                uint8_t get_id();
                uint8_t get_bus();
                std::string get_name(bool sensor);

                int get_pkg_loss()
                {
                   
                    return loss_var_;
                };
                void reset_pkg_loss()//(int cnt_lim, int ep_cnt)
                {
                    loss_var_ = packet_loss_;
                    // static_cast<double>(loss_var_ * cnt_lim * ep_cnt + packet_loss_ ) / 
                    // static_cast<double>(cnt_lim * (ep_cnt + 1));
                    packet_loss_ = 0;
                };
                bool get_msg_arrived()
                {
                    return msg_complete_;
                };
                void print_pl()
                {
                   RCLCPP_WARN(
                        rclcpp::get_logger("PP"),
                        "motor %d pkg loss at get is %d"
                        ,id_,packet_loss_); 
                };
                // Motor_Maneger(const Motor_Manager & other) = default;
                // Motor_Maneger(Motor_Manager && other) = default;

                

            private:
                // Command cmd_data_;
                // motor_info msr_data_;

                moteus::PositionResolution cmd_res_;
                moteus::QueryCommandV2 qry_res_;

                std::vector<Reply>* replies_;

                bool msg_valid_ = true;
                bool msg_complete_ = true;

                Get_Function get_callback_;
                Bind_Pol_Function pol_callback_;


                double msr_pos_ = 0.0;
                double msr_vel_= 0.0;
                double msr_trq_= 0.0;
                double msr_tmp_ = 0.0;
                double msr_enc_pos_ = 0.0;
                double msr_enc_vel_ = 0.0;
                double cmd_kp_scale_ = 1.0;
                double cmd_kd_scale_ = 1.0;
                double cmd_pos_ = 0.0;
                double cmd_vel_ = 0.0;
                double cmd_trq_ = 0.0;
                double msr_cur_ = 0.0;
                double max_trq_ = 0.0;
                double elect_power_ = 0.0;
                bool low_sat_ = false, high_sat_ = false;
                Command* cmd_data_;
                
                uint8_t id_;
                uint8_t bus_;
                double mot_red_ = 0.0,sens_red_=0.0, motor_trans_,sec_enc_trans_;
                std::string name_;
                std::vector<std::string> inter_type_stt_; 
                std::vector<std::string> inter_type_cmd_; 
                int packet_loss_ = 0;
                double loss_var_ = 0.0;
                double sec_enc_off_ = 0.0, old_sec_enc_ = 0.0;
                bool first_read_ = true;
                int sec_enc_counter_ = 0;
                double p_max_;
                double p_min_;
                double p_offset_;
                


        };
    }
}
#endif