#include "pi3hat_hw_interface/motor_manager.hpp"
#include "moteus_pi3hat/moteus_protocol.h"
#include "moteus_pi3hat/pi3hat.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <stdio.h>
#include <cstdio>

#define DELTA 0.5

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
            else if(type == hardware_interface::HW_IF_KP_SCALE ) 
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
                else if(type == hardware_interface::HW_IF_CURRENT)
                    return &msr_cur_;
                else if(type == hardware_interface::HW_IF_PACKAGE_LOSS)
                    return &loss_var_;
                else
                    throw std::invalid_argument("The passsed interface state type has not the correct type");
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
                    throw std::invalid_argument("The passsed interface state type has not the correct type");
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
            double pos_des;
            if(msg_valid_)
            {
                cmd_data_ -> id = id_;
                cmd_data_ -> bus = bus_;
                cmd_data_ -> mode = moteus::Mode::kPosition;
                if(low_sat_ && !high_sat_)
                {
                    cmd_data_ -> position.position = (p_min_* motor_trans_) / (2*M_PI) ;
                    cmd_data_ -> position.velocity = 0;
                    cmd_data_ -> position.feedforward_torque = 0;
                }
                else if( high_sat_ && !low_sat_)
                {
                    cmd_data_ -> position.position = (p_max_* motor_trans_) / (2*M_PI) ;
                    cmd_data_ -> position.velocity = 0;
                    cmd_data_ -> position.feedforward_torque = 0;
                }
                else if(!high_sat_ && !low_sat_)
                {
                    cmd_data_ -> position.position = ((cmd_pos_ - p_offset_) * motor_trans_) / (2*M_PI) ;
//                    RCLCPP_INFO(rclcpp::get_logger("motor_manager"),"the commanded pos of joint %d is %f",id_,cmd_data_->position.position);
                    cmd_data_ -> position.velocity =  (cmd_vel_ * motor_trans_ )/(2*M_PI);
                    cmd_data_ -> position.feedforward_torque = cmd_trq_ / motor_trans_;
                }
                else
                    assert(false);
                
               
                cmd_data_ -> position.kd_scale = cmd_kd_scale_;
                cmd_data_ -> position.kp_scale = cmd_kp_scale_;
                cmd_data_ -> query = qry_res_;
                cmd_data_ -> resolution = cmd_res_;

                // RCLCPP_INFO(rclcpp::get_logger("LIV"),"Quesy resolution encoder is %d %d and i pass %d %d",
                // cmd_data_->query.sec_enc_pos,
                // cmd_data_->query.sec_enc_vel,
                // qry_res_.sec_enc_pos,
                // qry_res_.sec_enc_vel);

                pol_callback_(msg_valid_,msg_complete_);
            }
            else
                RCLCPP_WARN(rclcpp::get_logger("pp"),"Occio non passa nell\'iff");
                
             
                // RCLCPP_INFO(rclcpp::get_logger("LIV50"),"value of data is %d",cmd_data_->query.sec_enc_pos);
                // RCLCPP_INFO(rclcpp::get_logger("LIV90"),"value of desidered data is %d",qry_res_.sec_enc_pos);
                
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
            double diff;
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
                msr_pos_ = (res.position/motor_trans_)*2*M_PI + p_offset_;
                if(msr_pos_ < p_min_ && p_min_ != 0.0)
                    low_sat_ = true;
                else if (msr_pos_ > p_max_ && p_max_ != 0.0) 
                    high_sat_ = true;
                else
                {
                    if(msr_pos_ < p_max_*0.95)
                        high_sat_ = false;
                    if(msr_pos_ > p_min_ * 0.95)
                        low_sat_ = false;
                }
                msr_vel_ = (res.velocity/motor_trans_)*2*M_PI;
                msr_trq_ = res.torque*motor_trans_;
                msr_tmp_ = res.temperature;
                msr_cur_ = res.q_current;

                if(sec_enc_trans_ != 0.0)
                {
                    
                    if(first_read_)
                    {
                        sec_enc_off_ = res.sec_enc_pos;
                        first_read_ = false;
                    }    
                    msr_enc_pos_ = res.sec_enc_pos - sec_enc_off_;
                    msr_enc_vel_ = (res.sec_enc_vel/sec_enc_trans_)*2*M_PI;
                    diff = msr_enc_pos_ - old_sec_enc_;
                    old_sec_enc_ = msr_enc_pos_;
                    if(diff > 0 && std::abs(diff) > DELTA)
                        sec_enc_counter_ --;
                    else if (diff < 0 &&  std::abs(diff) > DELTA)
                        sec_enc_counter_ ++;
                    msr_enc_pos_ += (float)sec_enc_counter_;
                    msr_enc_pos_ /= sec_enc_trans_;
                    msr_enc_pos_ *= 2*M_PI;
                    
                    // RCLCPP_INFO(rclcpp::get_logger("DIO"),"we have m: %f and se: %f computed by msr_wo:%f,msr_wo_o:%f and count:%d",
                    // msr_pos_,msr_enc_pos_,res.sec_enc_pos-sec_enc_off_,res.sec_enc_pos,sec_enc_counter_);
                }

                return res.fault;
            } 
        }


    }
}

