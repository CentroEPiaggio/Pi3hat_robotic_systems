#include "pi3hat_hw_interface/motor_manager.hpp"

#include <stdio.h>

namespace pi3hat_hw_interface
{
    namespace motor_manager
    {
        uint8_t Motor_Manager::get_id()
        {
            return id_;
        }
        uint8_t Motor_Manager::get_bus()
        {
            return bus_;
        }
        moteus::QueryCommandV2 Motor_Manager::get_qry_res()
        {
            return qry_res_;
        }
        moteus::PositionResolution Motor_Manager::get_cmd_res()
        {
            return cmd_res_;
        }
        void Motor_Manager::set_query_resolution(moteus::QueryCommandV2 res)
        {
            qry_res_ = res;
            if (!second_enc_)
            {
                qry_res_.sec_enc_pos = moteus::Resolution::kIgnore;
                qry_res_.sec_enc_vel = moteus::Resolution::kIgnore;
            }
        };

        void Motor_Manager::set_command_resolution(moteus::PositionResolution res)
        {
            cmd_res_ = res;
        }

        void Motor_Manager::make_position()
        {
            cmd_data_ -> id = id_;
            cmd_data_ -> bus = bus_;
            cmd_data_ -> mode = moteus::Mode::kPosition;

            cmd_data_ -> position.position = *cmd_pos_;
            cmd_data_ -> position.velocity = * cmd_vel_;
            cmd_data_ -> position.feedforward_torque = * cmd_trq_;
            cmd_data_ -> position.kd_scale = * cmd_kd_scale_;
            cmd_data_ -> position.kp_scale = * cmd_kp_scale_;
            cmd_data_ -> query = qry_res_;
            cmd_data_ -> resolution = cmd_res_;
            pol_callback_(msg_valid_,msg_complete_);
            
            
        }
        /* example of policy function 
        []
        
        */
        void Motor_Manager::make_stop()
        {
            cmd_data_ -> id = id_;
            cmd_data_ -> bus = bus_;
            cmd_data_ -> mode = moteus::Mode::kStopped;
        }
        void Motor_Manager::drop_torque()
        {
            cmd_data_ -> position.feedforward_torque = 0.0;
        }
        
        int Motor_Manager::get_motor_state()
        {
            int error;
            moteus::QueryResultV2 res = get_callback_(*replies_,bus_,id_,0,error);
            if(error == 1)
            {
                msg_valid_ = false;
                msg_complete_ = false;
                return 1;
            }
            else if( error == 2)
            {
                msg_valid_ = true;
                msg_complete_ = false;
                return 2;
            }
            else
            {
                msg_valid_ = true;
                msg_complete_ = true;
                *msr_pos_ = res.position;
                *msr_vel_ = res.velocity;
                *msr_trq_ = res.torque;
                *msr_tmp_ = res.temperature;
                if(second_enc_)
                {
                    *msr_enc_pos_ = res.sec_enc_pos;
                    *msr_enc_vel_ = res.sec_enc_vel;
                }

                return res.fault;
            }
            


           
        }


    }
}

