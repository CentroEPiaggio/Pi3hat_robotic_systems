#include "pi3hat_hw_interface/moteus_pi3hat_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pi3hat_hw_interface/elem_info_parsers.hpp"
#include <cmath>

#define LOGGER_NAME "MoteusPi3Hat_Interface"
#define CPU 1
using namespace rclcpp;
#define G 9.81
namespace pi3hat_hw_interface
{
    namespace moteus_pi3hat_interface
    {

         
        MoteusPi3Hat_Interface::MoteusPi3Hat_Interface()
        {
            
        };
        MoteusPi3Hat_Interface::~MoteusPi3Hat_Interface()
        {
           



           
        };

        CallbackReturn MoteusPi3Hat_Interface::on_init(const hardware_interface::HardwareInfo & info)
        {
            //create the parser objects
            std::unique_ptr<Pi3hatConfInfo> pi3hat_parser = std::make_unique<Pi3hatConfInfo>();
            std::unique_ptr<QueryFormatInfo> query_parser = std::make_unique<QueryFormatInfo>();
            std::unique_ptr<ActuatorConf> actuator_parser = std::make_unique<ActuatorConf>();
            std::string jnt_name;
            unsigned int bus,id, se_source;
            pi3hat_hw_interface::actuator_manager::ActuatorOptions act_opt;

            // parse  the option from the info
            try
            {
                pi3hat_parser->parse_map(info.hardware_parameters);
                pi3hat_parser->get_servo_map(info);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Pi3hat Transport parsing extra throw: %s",e.what());
                return CallbackReturn::FAILURE;
            }

            pi3hat_transport_ = std::make_shared<mjbots::pi3hat::Pi3HatMoteusTransport>(pi3hat_parser->get_cofigurable());
            if(pi3hat_parser->get_cofigurable().default_input.request_attitude)
            {
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Pi3hat IMU Attitude Data Requested");
                attittude_requested_ = true;
            }
            // get joints num and allocate the structures
            num_actuators_ = info.joints.size();
            actuators_.resize(num_actuators_);
            package_loss_.resize(num_actuators_);
            command_framees_.resize(num_actuators_);
            replies_.resize(num_actuators_*2);
            for(unsigned int i = 0; i < num_actuators_; i++)
            {
                // allocate actuator manager
                actuators_[i] = std::make_unique<pi3hat_hw_interface::actuator_manager::Actuator_Manager>(&command_framees_[i]);
                jnt_name = info.joints[i].name;
                try
                {
                    bus = std::stoi(info.joints[i].parameters.at("bus"));
                    id = std::stoi(info.joints[i].parameters.at("id"));
                    se_source = std::stoi(info.joints[i].parameters.at("second_encoder_source"));
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Parsing jnt %s BUS&ID throw: %s",jnt_name.c_str(),e.what());
                    return CallbackReturn::FAILURE;
                }
               
                // parse actuator parameters
                try
                {
                   actuator_parser->parse_map(info.joints[i].parameters);
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Actuator Configuration Parsing throw: %s",e.what());
                    return CallbackReturn::FAILURE;
                }
                
                
                act_opt = actuator_parser->get_cofigurable();
                actuators_[i]->SetActuatorParam(
                    act_opt,
                    id,
                    bus,
                    jnt_name
                );
                // parse query format
                try
                {
                    query_parser->parse_map(info.joints[i].parameters);
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Query Format Parsing throw: %s",e.what());
                    return CallbackReturn::FAILURE;
                }
                actuators_[i]->setSecondEncoderSource(se_source);
                actuators_[i]->setQueryFormat(query_parser->get_cofigurable());
                

            }

            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_configure(const rclcpp_lifecycle::State& )
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Start Actuator Configuration Procedure");
            for(unsigned int i = 0; i < num_actuators_; i++)
            {
                try
                {
                    actuators_[i]->ConfigureActuator(pi3hat_transport_);
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Actuator %s Configuration throw: %s",actuators_[i]->get_joint_name().c_str(),e.what());
                    return CallbackReturn::FAILURE;
                }
            }
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_cleanup(const rclcpp_lifecycle::State&)
        {
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_activate(const rclcpp_lifecycle::State&)
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Start Actuator Activation Procedure");
            for(unsigned int i = 0; i < num_actuators_; i++)
            {
                actuators_[i]->MakeStop();
            }
            
            pi3hat_transport_->BlockingCycle(
                command_framees_.data(),
                command_framees_.size(),
                &replies_
            );
            
            
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_deactivate(const rclcpp_lifecycle::State&)
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Start Actuator Deactivation Procedure");
            for(unsigned int i = 0; i < num_actuators_; i++)
            {
                actuators_[i]->MakeStop();
            }
            
            pi3hat_transport_->BlockingCycle(
                command_framees_.data(),
                command_framees_.size(),
                &replies_
            );
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_shutdown(const rclcpp_lifecycle::State&)
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Start Actuator Shuthdown Procedure");
            for(unsigned int i = 0; i < num_actuators_; i++)
            {
                actuators_[i]->MakeStop();
            }
            
            pi3hat_transport_->BlockingCycle(
                command_framees_.data(),
                command_framees_.size(),
                &replies_
            );

            return CallbackReturn::SUCCESS;
        };

         CallbackReturn MoteusPi3Hat_Interface::on_error(const rclcpp_lifecycle::State&)
        {
            
            return CallbackReturn::SUCCESS;
        };

        std::vector<hardware_interface::StateInterface> MoteusPi3Hat_Interface::export_state_interfaces()
        {   
            std::vector<hardware_interface::StateInterface> stt_int;
            if(attittude_requested_)
            {
                stt_int.emplace_back(
                    hardware_interface::StateInterface(
                        "pi3hat_hw_interface::imu","linear_acceleration.x",
                        &imu_linear_acceleration_.x
                    )
                );
                stt_int.emplace_back(
                    hardware_interface::StateInterface(
                        "pi3hat_hw_interface::imu","linear_acceleration.y",
                        &imu_linear_acceleration_.y
                    )
                );
                stt_int.emplace_back(
                    hardware_interface::StateInterface(
                        "pi3hat_hw_interface::imu","linear_acceleration.z",
                        &imu_linear_acceleration_.z
                    )
                );
                stt_int.emplace_back(
                    hardware_interface::StateInterface(
                        "pi3hat_hw_interface::imu","angular_velocity.x",
                        &imu_angular_velocity_.x
                    )
                );
                stt_int.emplace_back(
                    hardware_interface::StateInterface(
                        "pi3hat_hw_interface::imu","angular_velocity.y",
                        &imu_angular_velocity_.y
                    )
                );
                stt_int.emplace_back(
                    hardware_interface::StateInterface(
                        "pi3hat_hw_interface::imu","angular_velocity.z",
                        &imu_angular_velocity_.z
                    )
                );
                stt_int.emplace_back(
                    hardware_interface::StateInterface(
                        "pi3hat_hw_interface::imu","orientation.w",
                        &imu_orientation_.w
                    )
                );
                stt_int.emplace_back(
                    hardware_interface::StateInterface(
                        "pi3hat_hw_interface::imu","orientation.x",
                        &imu_orientation_.x
                    )
                );
                stt_int.emplace_back(
                    hardware_interface::StateInterface(
                        "pi3hat_hw_interface::imu","orientation.y",
                        &imu_orientation_.y
                    )
                );
                stt_int.emplace_back(   
                    hardware_interface::StateInterface(
                        "pi3hat_hw_interface::imu","orientation.z",
                        &imu_orientation_.z
                    )
                );
            }
            stt_int.emplace_back(
                hardware_interface::StateInterface(
                    "pi3hat_hw_interface::pi3hat","invalid_cycle",
                    &invalid_cycle_
                )
            );
            for(unsigned int i = 0; i < num_actuators_; i++)
            {
                actuators_[i]->ExportSttInt(stt_int);
                stt_int.emplace_back(
                    hardware_interface::StateInterface(
                        actuators_[i]->get_joint_name(),"package_loss",
                        &package_loss_[i]
                    )
                );
            }
            return stt_int;
        };
        
        std::vector<hardware_interface::CommandInterface> MoteusPi3Hat_Interface::export_command_interfaces()
        {
           std::vector<hardware_interface::CommandInterface> cmd_int;
           for(unsigned int i = 0; i < num_actuators_; i++)
           {
               actuators_[i]->ExportCmdInt(cmd_int);
           }
            return cmd_int;
        };

        hardware_interface::return_type MoteusPi3Hat_Interface::read(const rclcpp::Time & , const rclcpp::Duration & ) 
        {
            int p;
            if(!first_cycle_)
            {   
                if(clb_as_.try_consume() == -1)
                {
                    // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "read callback result obtained");
                    // RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),"Read Cycle Timeout");
                    invalid_cycle_ = 1.0;
                    for(unsigned int i = 0; i < num_actuators_; i++)
                    {
                        package_loss_[i] = 1.0;
                    }
                    return hardware_interface::return_type::OK;
                }
                else
                {
                    
                    invalid_cycle_ = 0.0;
                    //parse Imu information
                    if(attittude_requested_)
                    {
                        imu_orientation_ = filtered_IMU_.attitude;
                        imu_angular_velocity_ = filtered_IMU_.rate_dps;
                        imu_angular_velocity_.x *= (M_PI/180.0);
                        imu_angular_velocity_.y *= (M_PI/180.0);
                        imu_angular_velocity_.z *= (M_PI/180.0);
                        imu_linear_acceleration_ = filtered_IMU_.accel_mps2;
                    }
                    for(unsigned int i = 0; i < num_actuators_; i++)
                    {
                        package_loss_[i] = 1.0;
                    }
                    
                    for(auto rep: replies_)
                    {
                        for(unsigned int i = 0; i < num_actuators_; i++)
                        {
                            if(rep.source == actuators_[i]->GetActuatorId())
                            {
                                actuators_[i]->ParseSttFromReply(rep);
                                package_loss_[i] = 0.0;
                                break;
                            }
                        }
                    }
                }
            }
            else
            {
                // RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "skipping data processing");
                invalid_cycle_ = 0.0;
                first_cycle_ = false;
            }

            return hardware_interface::return_type::OK;
        };

        hardware_interface::return_type MoteusPi3Hat_Interface::write(const rclcpp::Time & , const rclcpp::Duration & ) 
        {
            if(invalid_cycle_ == 0.0)
            {
                // RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "send_data");
                for(unsigned int i = 0; i < num_actuators_; i++)
                {
                    actuators_[i]->MakeCommand();
                    // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Command for actuator %d %d",
                    //     command_framees_[i].expected_reply_size ,command_framees_[i].reply_required
                    // );
                }
                pi3hat_transport_->Cycle(
                    command_framees_.data(),
                    command_framees_.size(),
                    &replies_,
                    &filtered_IMU_,
                    nullptr,
                    nullptr,
                    clb_as_.callback()
                );
            }
            return hardware_interface::return_type::OK;
        };


    }
}PLUGINLIB_EXPORT_CLASS(
  pi3hat_hw_interface::moteus_pi3hat_interface::MoteusPi3Hat_Interface, hardware_interface::SystemInterface)
