#ifndef ELEM_INFO_PARSER_HPP
#define ELEM_INFO_PARSER_HPP
// GP include 
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <chrono>
#include <future>
#include <functional>
#include <stdexcept>
// moteus include 
#include "moteus_pi3hat/moteus.h"
#include "moteus_pi3hat/pi3hat_moteus_transport.h"
//ROS2 include
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
// ROS2 Control inlcude
#include "hardware_interface/hardware_info.hpp"

#pragma once
#define MAX_EXTRAS 5
#define MAX_VOLT 34
#define MAX_CURR 80
#define FLUX_BRAKE_MARGIN 27.5
#define MAX_POWER 450
namespace pi3hat_hw_interface
{
    namespace power_dist_manager
    {
        struct DistributorQuery
        {
            int state = 0;
            int switch_status = 0;
            int torque = 0;
            int lock_time = 0;
            int boot_time = 0;
            int voltage = 0;
            int current = 0;
            int temperature = 0;
            int energy = 0;
        };
    }
    namespace actuator_manager
    {
        struct ActuatorOptions
        {
            double Kp = 0.0;
            double Kd = 0.0;
            double Ki = 0.0;
            double ilimit = 0.0;
            double iratelimit = 0.0;
            double max_position_slip = 0.0;
            double max_velocity_slip = 0.0;
            double max_voltage = MAX_VOLT;
            double max_power_W = MAX_POWER;
            bool enable_motor_temperature = false;
            double max_current_A = MAX_CURR;
            double flux_brake_margin_voltage = FLUX_BRAKE_MARGIN;
            double actuator_transmission = 0.0;
            double second_encoder_trasmission = 0.0;
            double pos_max_limit = 0.0;
            double pos_min_limit = 0.0;
            double max_velocity = 0.0;
            double max_effort = 0.0;
            double position_offset = 0.0;
        };
        struct ActuatorQuery
        {

           int position = 0;
           int velocity = 0;
           int torque = 0;
           int q_current = 0;
           int d_current = 0;
           int abs_position = 0;
           int power = 0;
           int motor_temperature = 0;
           int voltage = 0;
           int temperature = 0;
           int position_error = 0;
           int velocity_error = 0;
           int torque_error = 0;
           int second_encoder_position = 0;
           int second_encoder_velocity = 0;
        };
    }
    using Resolution = mjbots::moteus::Resolution;
    template<typename T>
    class ElementInfo
    {
        public:
            T get_cofigurable()
            {
                return configurable_;
            }
            virtual void parse_map(std::unordered_map<std::string, std::string> pars) = 0;
            
        protected:
            //private funtion
            void check_unique(std::string param_name)
            {
                for(std::vector<std::string>::iterator i = parsed_params_.begin(); i != parsed_params_.end(); i++)
                {
                    if(*i == param_name)
                        throw std::runtime_error("param " + param_name + " is not unique");
                }
            }
            void check_existance(std::string param_name)
            {
                for(std::vector<std::string>::iterator i = available_base_params_.begin(); i != available_base_params_.end(); i++)
                {
                    if(*i == param_name)
                    return;
                }
                for(std::vector<std::string>::iterator i = available_extra_params_.begin(); i != available_extra_params_.end(); i++)
                {
                    if(*i == param_name)
                    return;
                }
                throw std::runtime_error("param " + param_name + " is listed into the available parameter");
            }
           
            //private member  
            T configurable_;
            std::vector<std::string> available_base_params_ = {};
            std::vector<std::string> available_extra_params_ = {};
            std::vector<std::string> parsed_params_ = {};  
    };

    class QueryFormatInfo : public ElementInfo<actuator_manager::ActuatorQuery>
    {
        
        public:
            // base 
            QueryFormatInfo()
            {

                // // init default query format
                // this->configurable_.mode = Resolution::kIgnore;
                // this->configurable_.position = Resolution::kInt32;
                // this->configurable_.velocity = Resolution::kInt32;
                // this->configurable_.torque = Resolution::kInt32;
                // this->configurable_.voltage = Resolution::kIgnore;
                // this->configurable_.temperature = Resolution::kInt16;
                // this->configurable_.motor_temperature = Resolution::kIgnore;
                // for(int i = 0; i < MAX_EXTRAS; i++)
                // {
                //     this->configurable_.extra[i].register_number = mjbots::moteus::detail::numeric_limits<int16_t>::max();
                //     this->configurable_.extra[i].resolution = Resolution::kIgnore;
                // }
                //init available params
                this->available_base_params_ = 
                {
                    "position_res",
                    "velocity_res",
                    "torque_res",
                    "q_current_res",
                    "d_current_res",
                    "abs_position_res",
                    "power_res",
                    "motor_temperature_res",
                    "voltage_res",
                    "temperature_res"
                };
                this->available_extra_params_ = 
                {
                    "position_error_res",
                    "velocity_error_res",
                    "torque_error_res",
                    "second_encoder_position_res",
                    "second_encoder_velocity_res",
                    
                };
                changes_.fill(false);
            }
            int parse_res(int res)
            {
                if(res == 0)
                    return 0;
                else if(res == 8)
                    return 1;
                else if(res == 16)
                    return 2;
                else if(res == 32)
                    return 3;
                else if(res == 64)
                    return 4;
                else
                    throw std::runtime_error("Wrong resolution format are available just [0,8,16,32,64] parsed " + std::to_string(res));
            }
            void parse_map(std::unordered_map<std::string,std::string> pars) override
            {
                // get second encoder source 
                try
                {
                   se_source_ = std::stoi(pars.at("second_encoder_source"));
                   if(se_source_ < 0 || se_source_ >2 )
                        throw std::runtime_error("second_encoder_source must be 0, 1 or 2");
                }
                catch(const std::exception& e)
                {
                    se_source_ = -1;
                }
                extra_count_ = 0;
                
                for(std::unordered_map<std::string,std::string>::iterator i = pars.begin(); i != pars.end(); i++)
                {
                    
                    this->check_unique(i->first);

                    if(i->first.compare(this->available_base_params_[0]) == 0)
                    {
                        configurable_.position = this->parse_res(std::stoi(i->second));
                        changes_[0] = true;
                    }
                    else if(i->first.compare(this->available_base_params_[1]) == 0)
                    {
                        configurable_.velocity = this->parse_res(std::stoi(i->second));
                        changes_[1] = true;
                    }
                    else if(i->first.compare(this->available_base_params_[2]) == 0)
                    {
                        configurable_.torque = this->parse_res(std::stoi(i->second));
                        changes_[2] = true;
                    }
                    else if(i->first.compare(this->available_base_params_[3]) == 0)
                    {
                        configurable_.q_current = this->parse_res(std::stoi(i->second));
                        changes_[3] = true;
                    }
                    else if(i->first.compare(this->available_base_params_[4]) == 0)
                    {
                        configurable_.d_current = this->parse_res(std::stoi(i->second));
                        changes_[4] = true;
                    }
                    else if(i->first.compare(this->available_base_params_[5]) == 0)
                    {
                        configurable_.abs_position = this->parse_res(std::stoi(i->second));
                        changes_[5] = true;
                    }
                    else if(i->first.compare(this->available_base_params_[6]) == 0)
                    {
                        configurable_.power = this->parse_res(std::stoi(i->second));
                        changes_[6] = true;
                    }
                    else if(i->first.compare(this->available_base_params_[7]) == 0)
                    {
                        configurable_.motor_temperature = this->parse_res(std::stoi(i->second));
                        changes_[7] = true;
                    }
                    else if(i->first.compare(this->available_base_params_[8]) == 0)
                    {
                        configurable_.voltage = this->parse_res(std::stoi(i->second));
                        changes_[8] = true;
                    }
                    else if(i->first.compare(this->available_base_params_[9]) == 0)
                    {
                        configurable_.temperature = this->parse_res(std::stoi(i->second));
                        changes_[9] = true;
                    }
                    else if(i->first.compare(this->available_extra_params_[0]) == 0 && std::stoi(i->second) != 0 )
                    {
                        configurable_.position_error = this->parse_res(std::stoi(i->second));
                        // if(extra_count_ < MAX_EXTRAS)
                        // {
                        //     configurable_.extra[extra_count_].resolution = parse_res(std::stoi(i->second));
                        //     RCLCPP_INFO(rclcpp::get_logger("Query Resolution Parser DEBUG"),"Parsing extra param %s with res %s, %d",i->first.c_str(),i->second.c_str(),parse_res(std::stoi(i->second)));
                        //     configurable_.extra[extra_count_].register_number = mjbots::moteus::Register::kControlPositionError;
                        //     RCLCPP_INFO(rclcpp::get_logger("Query Resolution Parser DEBUG"),"Extra %d : register number %d with res %d",extra_count_,configurable_.extra[extra_count_].register_number,static_cast<int>(configurable_.extra[extra_count_].resolution));
                        //     extra_count_ ++;
                        // }
                        // else
                        //     throw std::runtime_error("Reach maximum extras element");
                    }
                    else if(i->first.compare(this->available_extra_params_[1]) == 0 && std::stoi(i->second) != 0 ) 
                    {
                        configurable_.velocity_error = this->parse_res(std::stoi(i->second));
                        // if(extra_count_ < MAX_EXTRAS)
                        // {
                        //     configurable_.extra[extra_count_].resolution = parse_res(std::stoi(i->second));
                        //     configurable_.extra[extra_count_].register_number = mjbots::moteus::Register::kControlVelocityError;
                        //     RCLCPP_INFO(rclcpp::get_logger("Query Resolution Parser DEBUG"),"Extra %d : register number %d with res %d",extra_count_,configurable_.extra[extra_count_].register_number,static_cast<int>(configurable_.extra[extra_count_].resolution));
                        //     extra_count_ ++;
                        // }
                        // else
                        //     throw std::runtime_error("Reach maximum extras element");
                    }
                    else if(i->first.compare(this->available_extra_params_[2]) == 0 && std::stoi(i->second) != 0 )
                    {
                        configurable_.torque_error = this->parse_res(std::stoi(i->second));
                        // if(extra_count_ < MAX_EXTRAS)
                        // {
                        //     configurable_.extra[extra_count_].resolution = parse_res(std::stoi(i->second));
                        //     configurable_.extra[extra_count_].register_number = mjbots::moteus::Register::kControlTorqueError;
                        //     RCLCPP_INFO(rclcpp::get_logger("Query Resolution Parser DEBUG"),"Extra %d : register number %d with res %d",extra_count_,configurable_.extra[extra_count_].register_number,static_cast<int>(configurable_.extra[extra_count_].resolution));
                        //     extra_count_ ++;
                        // }
                        // else
                        //     throw std::runtime_error("Reach maximum extras element");
                    }
                    else if(i->first.compare(this->available_extra_params_[3]) == 0 && std::stoi(i->second) != 0 )
                    {
                        if(se_source_ == -1)
                        {
                            RCLCPP_WARN(rclcpp::get_logger("Query Resolution Parser"),"No second encoder will be used");
                        }
                        else
                            configurable_.second_encoder_position = this->parse_res(std::stoi(i->second));
                        // if(extra_count_ < MAX_EXTRAS)
                        // {
                        //     configurable_.extra[extra_count_].resolution = parse_res(std::stoi(i->second));
                        //     if(se_source_ == 0)
                        //         configurable_.extra[extra_count_].register_number = mjbots::moteus::Register::kEncoder0Position;
                        //     else if(se_source_ == 1)
                        //         configurable_.extra[extra_count_].register_number = mjbots::moteus::Register::kEncoder1Position;
                        //     else if(se_source_ == 2)
                        //         configurable_.extra[extra_count_].register_number = mjbots::moteus::Register::kEncoder2Position;
                        //     else
                        //         throw std::runtime_error("second_encoder_source must be 0, 1 or 2");
                        //     // RCLCPP_INFO(rclcpp::get_logger("Query Resolution Parser"),"Extra %d : register number %d with res %d",extra_count_,configurable_.extra[extra_count_].register_number,static_cast<int>(configurable_.extra[extra_count_].resolution));
                        //     extra_count_ ++;

                        // }
                        // else
                        //     throw std::runtime_error("Reach maximum extras element");
                    }
                    else if(i->first.compare(this->available_extra_params_[4]) == 0 && std::stoi(i->second) != 0 )
                    {
                        if(se_source_ == -1)
                        {
                            RCLCPP_WARN(rclcpp::get_logger("Query Resolution Parser"),"No second encoder will be used");
                        }
                        else
                            configurable_.second_encoder_velocity = this->parse_res(std::stoi(i->second));
                        // if(extra_count_ < MAX_EXTRAS)
                        // {
                        //     configurable_.extra[extra_count_].resolution = parse_res(std::stoi(i->second));
                        //     if(se_source_ == 0)
                        //         configurable_.extra[extra_count_].register_number = mjbots::moteus::Register::kEncoder0Velocity;
                        //     else if(se_source_ == 1)
                        //         configurable_.extra[extra_count_].register_number = mjbots::moteus::Register::kEncoder1Velocity;
                        //     else if(se_source_ == 2)
                        //         configurable_.extra[extra_count_].register_number = mjbots::moteus::Register::kEncoder2Velocity;
                        //     else
                        //         throw std::runtime_error("second_encoder_source must be 0, 1 or 2");
                        //     // RCLCPP_INFO(rclcpp::get_logger("Query Resolution Parser"),"Extra %d : register number %d with res %d",extra_count_,configurable_.extra[extra_count_].register_number,static_cast<int>(configurable_.extra[extra_count_].resolution));
                        //     extra_count_ ++;
                        // }
                        // else
                        //     throw std::runtime_error("Reach maximum extras element");
                    }
                    // else
                    // {
                    //     RCLCPP_WARN(rclcpp::get_logger("Query Resolution Parser"),"The param %s do not exist", i->first.c_str());
                    //     throw std::runtime_error("Unable to parse parameters");
                    // }
                }
               
                for(size_t i = 0; i<changes_.size(); i++)
                {
                    if(!changes_[i])
                        RCLCPP_WARN(rclcpp::get_logger("Query Resolution Parser"),"The base query conf %s is set to default", available_base_params_[i].c_str());
                }
            } 
            int get_secodn_encoder_source()
            {
                return se_source_;
            };
        private:
            std::array<bool,10> changes_ ;
            int extra_count_ = 0, se_source_;   
            
    };
    class Pi3hatConfInfo : public ElementInfo<mjbots::pi3hat::Pi3HatMoteusTransport::Options>
    {
        public:
            Pi3hatConfInfo()
            {
                this->available_base_params_ = 
                {
                    "timeout_ns",
                    "min_tx_wait_ns",
                    "rx_baseline_wait_ns",
                    "rx_extra_wait_ns",
                    "CPU_affinity",
                    "request_attitude"
                };
                this->available_extra_params_=
                {
                    "mounting_deg_roll",
                    "mounting_deg_pitch",
                    "mounting_deg_yaw",
                    "attitude_hz"
                };
                changes_.fill(false);
                
            };
            void get_servo_map(hardware_interface::HardwareInfo info)
            {
                if(servo_map_set_)
                    return;
                // set up servo map for trasport 
                for(auto jnt: info.joints)
                {
                    
                    try
                    {
                        auto res = configurable_.servo_map.insert({
                            std::stoi(jnt.parameters.at("id")),
                            std::stoi(jnt.parameters.at("bus"))
                        });
                        if(!res.second)
                            throw std::runtime_error("Error, motors id are not unique");
                    }
                    catch(const std::exception& e)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("Query Resolution Parser"),"Parsing jnt %s ID throw: %s",jnt.name.c_str(),e.what());
                        throw std::runtime_error("Error durning BUS&ID Parsing");
                    }
                }
                servo_map_set_ = true;
            }
            void parse_map(std::unordered_map<std::string,std::string> pars) override
            {
                for(std::unordered_map<std::string,std::string>::iterator i = pars.begin(); i != pars.end(); i++)
                {
                    this->check_unique(i->first);
                    
                    
                    if(i->first.compare(available_base_params_[0]) == 0)
                    {
                        configurable_.default_input.timeout_ns = std::stoi(i->second);
                        changes_[0] = true;
                    }
                    else if(i->first.compare(available_base_params_[1]) == 0)
                    {
                        configurable_.default_input.timeout_ns = std::stoi(i->second);
                        changes_[1] = true;
                    }
                    else if(i->first.compare(available_base_params_[2]) == 0)
                    {
                        configurable_.default_input.timeout_ns = std::stoi(i->second);
                        changes_[2] = true;
                    }
                    else if(i->first.compare(available_base_params_[3]) == 0)
                    {
                        configurable_.default_input.timeout_ns = std::stoi(i->second);
                        changes_[3] = true;
                    }
                    else if(i->first.compare(available_base_params_[4]) == 0)
                    {
                        configurable_.default_input.timeout_ns = std::stoi(i->second);
                        changes_[4] = true;
                    }
                    else if(i->first.compare(available_base_params_[5]) == 0)
                    {
                        configurable_.default_input.timeout_ns = std::stoi(i->second);
                        changes_[5] = true;
                    }
                    else if(i->first.compare(available_extra_params_[0]) == 0)
                        configurable_.mounting_deg.roll = std::stod(i->second);
                    else if(i->first.compare(available_extra_params_[1]) == 0)
                        configurable_.mounting_deg.pitch = std::stod(i->second);
                    else if(i->first.compare(available_extra_params_[2]) == 0)
                        configurable_.mounting_deg.yaw= std::stod(i->second);
                    else if(i->first.compare(available_extra_params_[3]) == 0)
                        configurable_.attitude_rate_hz = std::stoi(i->second);
                    else
                    {
                        RCLCPP_WARN(rclcpp::get_logger("Pi3Hat Transport Parser"),"The param %s do not exist", i->first.c_str());
                        throw std::runtime_error("Unable to parse parameters");
                    }  
                }
                for(size_t i = 0; i<changes_.size(); i++)
                {
                    if(!changes_[i])
                        RCLCPP_WARN(rclcpp::get_logger("Pi3Hat Transport Parser"),"The pi3hat  conf %s is set to default", this->available_base_params_[i].c_str());
                } 
        }
        private: 
            bool servo_map_set_ = false;
            std::array<bool,6> changes_;  
    };
    class ActuatorConf : public ElementInfo<actuator_manager::ActuatorOptions>
    {
        public: 
            ActuatorConf()
            {
                available_base_params_ =
                {
                    "KP",
                    "KD",
                    "KI",
                    "ilimit",
                    "iratelimit",
                    "max_position_slip",
                    "max_velocity_slip",
                    "enable_motor_temperature",
                    "min_pos_limit",
                    "max_pos_limit",
                    "max_velocity",
                    "max_effort",
                    "actuator_trasmission",
                    "position_offset"
                };
                available_extra_params_ =
                {
                    "second_encoder_trasmission"
                };
                changes_.fill(false);

            };
            void parse_map(std::unordered_map<std::string,std::string> pars) override
            {
                for(std::unordered_map<std::string,std::string>::iterator i = pars.begin(); i != pars.end(); i++)
                {
                    this->check_unique(i->first);
                    if(i->first.compare(available_base_params_[0]) == 0)
                    {
                        configurable_.Kp = std::stod(i->second);
                        changes_[0] = true;
                    }
                    else if(i->first.compare(available_base_params_[1]) == 0)
                    {
                        configurable_.Kd = std::stod(i->second);
                        changes_[1] = true;
                    }
                    else if(i->first.compare(available_base_params_[2]) == 0)
                    {
                        configurable_.Ki = std::stod(i->second);
                        changes_[2] = true;
                    }
                    else if(i->first.compare(available_base_params_[3]) == 0)
                    {
                        configurable_.ilimit = std::stod(i->second);
                        changes_[3] = true;
                    }
                    else if(i->first.compare(available_base_params_[4]) == 0)
                    {
                        configurable_.iratelimit = std::stod(i->second);
                        changes_[4] = true;
                    }
                    else if(i->first.compare(available_base_params_[5]) == 0)
                    {
                        configurable_.max_position_slip = std::stod(i->second);
                        changes_[5] = true;
                    }
                    else if(i->first.compare(available_base_params_[6]) == 0)
                    {
                        configurable_.max_velocity_slip = std::stod(i->second);
                        changes_[6] = true;
                    }
                    else if(i->first.compare(available_base_params_[7]) == 0)
                    {
                        configurable_.enable_motor_temperature = std::stod(i->second);
                        changes_[7] = true;
                    }
                    else if(i->first.compare(available_base_params_[8]) == 0)
                    {
                        configurable_.pos_min_limit = std::stod(i->second);
                        changes_[8] = true;
                    }
                    else if(i->first.compare(available_base_params_[9]) == 0)
                    {
                        configurable_.pos_max_limit = std::stod(i->second);
                        changes_[9] = true;
                    }
                    else if(i->first.compare(available_base_params_[10]) == 0)
                    {
                        configurable_.max_velocity = std::stod(i->second);
                        changes_[10] = true;
                    }
                    else if(i->first.compare(available_base_params_[11]) == 0)
                    {
                        configurable_.max_effort = std::stod(i->second);
                        changes_[11] = true;
                    }
                    else if(i->first.compare(available_base_params_[12]) == 0)
                    {
                        configurable_.actuator_transmission = std::stod(i->second);
                        changes_[12] = true;
                    }
                    else if(i->first.compare(available_base_params_[13]) == 0)
                    {
                        configurable_.position_offset = std::stod(i->second);
                        changes_[13] = true;
                    }
                    else if(i->first.compare(available_extra_params_[0]) == 0)
                        configurable_.second_encoder_trasmission = std::stod(i->second);  
                }
                for(size_t i = 0; i<changes_.size(); i++)
                {
                    if(!changes_[i])
                        RCLCPP_WARN(rclcpp::get_logger("Moteus Driver Param"),"The moteus conf %s is set to default", available_base_params_[i].c_str());
                } 
            };
        private:
            std::array<bool,14> changes_;
    };

    class PDQueryFormatInfo :  public ElementInfo<power_dist_manager::DistributorQuery>
    {
        public:
            PDQueryFormatInfo()
            {
                available_base_params_ =
                    {
                        "voltage",
                        "current",
                        "temperature",
                        "energy",
                        
                    };
                    available_extra_params_ =
                    {
                        "state",
                        "switch_state",
                        "lock_time",
                        "boot_time"
                    };
                    changes_.fill(false);
            }
            void parse_map(std::unordered_map<std::string,std::string> pars) override
            {
                for(std::unordered_map<std::string,std::string>::iterator i = pars.begin(); i != pars.end(); i++)
                {
                    this->check_unique(i->first);
                    if(i->first.compare(available_base_params_[0]) == 0)
                    {
                        configurable_.voltage =  this->parse_res(std::stod(i->second));
                        changes_[0] = true;
                    }
                    else if(i->first.compare(available_base_params_[1]) == 0)
                    {
                        configurable_.current =  this->parse_res(std::stod(i->second));
                        changes_[1] = true;
                    }
                    else if(i->first.compare(available_base_params_[2]) == 0)
                    {
                        configurable_.temperature =  this->parse_res(std::stod(i->second));
                        changes_[2] = true;
                    }
                    else if(i->first.compare(available_base_params_[3]) == 0)
                    {
                        configurable_.energy =  this->parse_res(std::stod(i->second));
                        changes_[3] = true;
                    }
                    else if(i->first.compare(available_extra_params_[0]) == 0)
                        configurable_.state = this->parse_res(std::stod(i->second));  
                    else if(i->first.compare(available_extra_params_[1]) == 0)
                        configurable_.state = this->parse_res(std::stod(i->second));
                    else if(i->first.compare(available_extra_params_[2]) == 0)
                        configurable_.state = this->parse_res(std::stod(i->second));
                    else if(i->first.compare(available_extra_params_[3]) == 0)
                        configurable_.state = this->parse_res(std::stod(i->second));
                }
            }
        private:
            int parse_res(int res)
            {
                if(res == 0)
                    return 0;
                else if(res == 8)
                    return 1;
                else if(res == 16)
                    return 2;
                else if(res == 32)
                    return 3;
                else if(res == 64)
                    return 4;
                else
                    throw std::runtime_error("Wrong resolution format are available just [0,8,16,32,64] parsed " + std::to_string(res));
            };
            std::array<bool,4> changes_;
    };
};
#endif