#include "moteus_pi3hat/moteus_protocol.h"
#include "moteus_pi3hat/realtime.h"
#include "moteus_pi3hat/pi3hat_moteus_interface.h"
#include <string>
#include <cstdint>
#include <vector>
#include <functional>
// #define NULL __null

using namespace mjbots;
using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using MoteusInterface = moteus::Pi3HatMoteusInterface;
using Command = moteus::Pi3HatMoteusInterface::ServoCommand;
using Reply = moteus::Pi3HatMoteusInterface::ServoReply;
using Get_Function = std::function<moteus::QueryResultV2 ( std::vector<Reply>& replies, int bus, int id, int opt,int& err)>;
using Policy_Function = std::function<void(bool msg_valid, bool msg_coplete, Command* cmd_d)>;
using Bind_Pol_Function = std::function<void(bool msg_valid, bool msg_coplete)>;

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
            double* msr_pos = NULL;
            double* msr_vel = NULL;
            double* msr_trq = NULL;
            double* msr_tmp = NULL;
            double* msr_enc_pos = NULL;
            double* msr_enc_vel = NULL;
            double* cmd_pos = NULL;
            double* cmd_vel = NULL;
            double* cmd_trq = NULL;
            double* kp_scale = NULL;
            double* kd_scale = NULL;
            Command* cmd_data = NULL;
            std::vector<Reply>* replies = NULL;
            

        };




        class Motor_Manager
        {
            public:
                Motor_Manager(
                    motor_mem mem_str, 
                    bool second_enc, 
                    uint8_t id, 
                    uint8_t bus, 
                    Policy_Function pl_fun,
                    Get_Function gt_fun):
                second_enc_(second_enc),
                id_(id),
                bus_(bus)
                {
                    msr_pos_ = mem_str.msr_pos;
                    msr_vel_ = mem_str.msr_vel;
                    msr_trq_ = mem_str.msr_trq;
                    msr_tmp_ = mem_str.msr_tmp;
                    cmd_kp_scale_ = mem_str.kp_scale;
                    cmd_kd_scale_ = mem_str.kd_scale;
                    cmd_data_ = mem_str.cmd_data;
                    cmd_pos_ = mem_str.cmd_pos;
                    cmd_vel_ = mem_str.cmd_vel;
                    cmd_trq_ = mem_str.cmd_trq;
                    replies_ = mem_str.replies;
                    if(second_enc)
                    {
                        msr_enc_pos_ = mem_str.msr_enc_pos;
                        msr_enc_vel_ = mem_str.msr_enc_vel;
                    }
                    // std::function<void(int,int,int)> F([](int a, int b, int c){
                        
                    //     b--;
                    //     c = a;
                    //     a++;
                    // });
                    pol_callback_ = std::bind(pl_fun,_1,_2,this->cmd_data_);
                    get_callback_ = gt_fun;
                    //std::function<void(bool,bool,Command&)> a = bind(&pl_fun,this, _1,_2,_3);
                };
                ~Motor_Manager()
                {};
                // set the current message resolution as res in cmd_data_
                void set_command_resolution(moteus::PositionResolution res);

                moteus::QueryCommandV2 get_qry_res();
                moteus::PositionResolution get_cmd_res();

                void set_query_resolution(moteus::QueryCommandV2 res);

                // set the command position into the data structure
                void make_position();

                // set the command stop to all motors
                void make_stop();

                void drop_torque();

                // get the readed value into state_vars and set flag to decide 
                int get_motor_state();

                uint8_t get_id();
                uint8_t get_bus();

                

            private:
                // Command cmd_data_;
                // motor_info msr_data_;

                moteus::PositionResolution cmd_res_;
                moteus::QueryCommandV2 qry_res_;

                std::vector<Reply>* replies_;

                bool msg_valid_;
                bool msg_complete_;

                Get_Function get_callback_;
                Bind_Pol_Function pol_callback_;


                double* msr_pos_;
                double* msr_vel_;
                double* msr_trq_;
                double* msr_tmp_;
                double* msr_enc_pos_;
                double* msr_enc_vel_;
                double* cmd_kp_scale_;
                double* cmd_kd_scale_;
                double* cmd_pos_;
                double* cmd_vel_;
                double* cmd_trq_;
                Command* cmd_data_;
                bool second_enc_;
                uint8_t id_;
                uint8_t bus_;

                


        };
    }
}
