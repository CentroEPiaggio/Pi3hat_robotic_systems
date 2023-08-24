#include "pi3hat_hw_interface/motor_manager.hpp"
#include "moteus_pi3hat/moteus_protocol.h"
#include "moteus_pi3hat/pi3hat.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include <stdio.h>
#include <cstdio>

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

        std::string Motor_Manager::get_name(bool sensor)
        {
            if(!sensor)
                return name_;
            else if(sensor && sec_enc_trans_!=0.0)
            {
                return name_+"_ENCODER1";
            }
            else
                throw std::logic_error("You can not have second ecoder msr data id sec_enc_transmission is 0.0");
        }

        moteus::QueryCommandV2 Motor_Manager::get_qry_res()
        {
            return qry_res_;
        }

        moteus::PositionResolution Motor_Manager::get_cmd_res()
        {
            return cmd_res_;
        }

        double* Motor_Manager::get_cmd_interface(std::string type)
        {
           
            if(type == hardware_interface::HW_IF_POSITION)
                return &cmd_pos_;
            else if(type == hardware_interface::HW_IF_VELOCITY)
                return &cmd_vel_;
            else if(type == hardware_interface::HW_IF_EFFORT)
                return &cmd_trq_;
            else if(type == hardware_interface::HW_IF_KP_SCALE  ) 
                return &cmd_kp_scale_;
            else if(type == hardware_interface::HW_IF_KD_SCALE)
                return &cmd_kd_scale_;

            else
                throw std::invalid_argument("The passsed interface command type has not the correct type");
        
            
        };

        double* Motor_Manager::get_stt_interface(std::string type, bool sensor)
        {
            if(!sensor)
            {
                //std::printf("1:::::\n");
                if(type == hardware_interface::HW_IF_POSITION)
                    return &msr_pos_;
                else if(type == hardware_interface::HW_IF_VELOCITY)
                    return &msr_vel_;
                else if(type == hardware_interface::HW_IF_EFFORT)
                    return &msr_trq_;
                else if(type == hardware_interface::HW_IF_TEMPERATURE)
                    return &msr_tmp_;
                else if(type == hardware_interface::HW_IF_PACKAGE_LOSS)
                    return &loss_var_;
                else
                    throw std::invalid_argument("The passsed interface command type has not the correct type");
            }
            if(sensor && sec_enc_trans_!= 0.0)
            {
                //std::printf("2:::::\n");
                if(type == hardware_interface::HW_IF_POSITION)
                    return &msr_enc_pos_;
                else if(type == hardware_interface::HW_IF_VELOCITY)
                {
                    //std::printf("vel:::::\n");
                    return &msr_enc_vel_;
                }
               else
                    throw std::invalid_argument("The passsed interface command type has not the correct type");
            }
            return 0;
            
        };
        void Motor_Manager::set_query_resolution(moteus::QueryCommandV2 res)
        {
            qry_res_ = res;
            if (sec_enc_trans_ == 0.0)
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
            if(msg_valid_)
            {
                cmd_data_ -> id = id_;
                cmd_data_ -> bus = bus_;
                cmd_data_ -> mode = moteus::Mode::kPosition;
                cmd_data_ -> position.position = cmd_pos_ * motor_trans_;
                cmd_data_ -> position.velocity =  cmd_vel_ * motor_trans_;
                cmd_data_ -> position.feedforward_torque = cmd_trq_ / motor_trans_;
                cmd_data_ -> position.kd_scale = cmd_kd_scale_;
                cmd_data_ -> position.kp_scale = cmd_kp_scale_;
                cmd_data_ -> query = qry_res_;
                cmd_data_ -> resolution = cmd_res_;
                pol_callback_(msg_valid_,msg_complete_);
            }
            else
                RCLCPP_WARN(rclcpp::get_logger("pp"),"Occio non passa nell\'iff");
                
            
        }

        void Motor_Manager::make_stop()
        {
            //RCLCPP_INFO(rclcpp::get_logger("pinO"),"IL VALORE DI ID prima DEL MK_STP E\' %d",cmd_data_->id);
            cmd_data_ -> id = id_;
            //RCLCPP_INFO(rclcpp::get_logger("pinO"),"IL VALORE DI ID dopo DEL MK_STP E\' %d",cmd_data_->id);
            cmd_data_ -> bus = bus_;
            cmd_data_ -> mode = moteus::Mode::kStopped;
            cmd_data_ ->query = qry_res_;
            cmd_data_ -> resolution = cmd_res_;
        }
        

        void Motor_Manager::drop_torque()
        {
            cmd_data_ -> position.feedforward_torque = 0.0;
        }
        
        int Motor_Manager::get_motor_state(int prov_msg)
        {
            int error;
            // count_++;
            // if(count_ % MAX_COUNT == 0)
            // {
            //     RCLCPP_WARN(rclcpp::get_logger("LOGGER_NAME"), "arrived msg percentage of motor ID::%d is %lf",id_, ((float)packet_loss_)/(float)count_);

            //     count_ = 0;
            //     packet_loss_ = 0;

            // }
            
            //std::printf("reply nums %d\n",replies_->size());
            moteus::QueryResultV2 res = get_callback_(*replies_,bus_,id_,msg_complete_,error,prov_msg);
           
            if(error == 1)
            {
                // assert(false);
                msg_complete_ = false;
                this->packet_loss_ ++;
                msr_pos_ = std::nan("1");
                msr_vel_ = std::nan("1");
                msr_trq_ = std::nan("1");
                msr_tmp_ = std::nan("1");
                if(sec_enc_trans_ != 0.0)
                {
                    msr_enc_pos_ = std::nan("1");
                    msr_enc_vel_ = std::nan("1");
                }
                return 1;
            }
            else
            {
                // RCLCPP_ERROR(rclcpp::get_logger("PP"),"the read is ok");
                msg_complete_ = true;
                msr_pos_ = res.position/motor_trans_;
                msr_vel_ = res.velocity/motor_trans_;
                msr_trq_ = res.torque*motor_trans_;
                msr_tmp_ = res.temperature;
                if(sec_enc_trans_ != 0.0)
                {
                    msr_enc_pos_ = res.sec_enc_pos/sec_enc_trans_;
                    msr_enc_vel_ = res.sec_enc_vel/sec_enc_trans_;
                }

                return res.fault;
            } 
        }


    }
}

