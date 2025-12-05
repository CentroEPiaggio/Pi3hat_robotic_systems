#include "pi3hat_hw_interface/actuator_manager.hpp"

namespace pi3hat_hw_interface
{
    namespace actuator_manager
    {
        
        using ControllerOptions = mjbots::moteus::Controller::Options;
        Actuator_Manager::~Actuator_Manager()
        {
            
        };

        bool Actuator_Manager::ConfigureActuator( std::shared_ptr<mjbots::moteus::Transport> transport)
        {
            ControllerOptions c_opt;
            QueryFormat qf;
            std::string res;
            // prepare query format
            qf.extra[0].register_number = mjbots::moteus::Register::kEncoderValidity;
            qf.extra[0].resolution = mjbots::moteus::Resolution::kFloat;
            // set up controller options
            c_opt.id = id_;
            c_opt.bus = bus_;
            c_opt.transport = transport;
            c_opt.position_format.position = Resolution::kInt32;
            c_opt.position_format.velocity = Resolution::kInt32;
            c_opt.position_format.feedforward_torque = Resolution::kInt32;
            c_opt.position_format.kd_scale = Resolution::kInt16;
            c_opt.position_format.kp_scale = Resolution::kInt16;
            actuator_transmission_ = act_opt_.actuator_transmission;
            second_encoder_transmission_ = act_opt_.second_encoder_trasmission;
            if(second_encoder_transmission_ > 0.0)
                second_encoder_output_ = std::make_unique<SecondEncoderOutput>(second_encoder_transmission_);
            max_torque_ = act_opt_.max_effort;
            position_offset_ = act_opt_.position_offset;
            
            //create controller 
            c_ = std::make_unique<Controller>(c_opt);

            // start configuration procedure flushing all stored information
            c_->DiagnosticWrite("d stop \n");
            c_->DiagnosticFlush();
            // get information about encoder validity
            auto result = c_->SetQuery(&qf);
            if(result.has_value())
            {
                int validity = static_cast<int>(result->values.extra[0].value);
                if(
                    se_source_ == 0 && 
                    (validity & (1<<0)) &&
                    (validity & (1<<1))
                )
                {
                    RCLCPP_INFO(rclcpp::get_logger("Actuator_Manager"),"Using first source to read the second encoder for actuator id %d on bus %d",id_,bus_);
                    RCLCPP_WARN(rclcpp::get_logger("Actuator_Manager"),"Usually source 0 is used for embedded encoder, pay attention!!");
                }
                else if(
                    se_source_ == 1 && 
                    (validity & (1<<2)) &&
                    (validity & (1<<3))
                )
                    RCLCPP_INFO(rclcpp::get_logger("Actuator_Manager"),"Using second source to read the second encoder for actuator id %d on bus %d",id_,bus_);
                
                else if(
                    se_source_ == 2 && 
                    (validity & (1<<4)) &&
                    (validity & (1<<5))
                )
                    RCLCPP_INFO(rclcpp::get_logger("Actuator_Manager"),"Using third source to read the second encoder for actuator id %d on bus %d",id_,bus_);
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("Actuator_Manager"), "Second encoder source and driver configuration are not coupled");
                    throw std::runtime_error("Wrong Configuration");
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("Actuator_Manager"),"Failed to get encoder validity information for actuator id %d on bus %d",id_,bus_);
                throw std::runtime_error("Failed to get encoder validity information");
            }
            // start motor configuration
            c_->DiagnosticWrite("conf set servo.pid_position.kp " + std::to_string(FromJointToMotorGain(act_opt_.Kp,true)) + "\n");
            c_->DiagnosticWrite("conf set servo.pid_position.kd " + std::to_string(FromJointToMotorGain(act_opt_.Kd,true)) + "\n");
            c_->DiagnosticWrite("conf set servo.pid_position.ki " + std::to_string(FromJointToMotorGain(act_opt_.Ki,true)) + "\n");
            c_->DiagnosticWrite("conf set servo.pid_position.ilimit " + std::to_string(FromJointToMotorEffort(act_opt_.ilimit,true)) + "\n");
            c_->DiagnosticWrite("conf set servo.pid_position.iratelimit " + std::to_string(FromJointToMotorEffort(act_opt_.iratelimit,true)) + "\n");
            c_->DiagnosticWrite("conf set servo.max_position_slip " + std::to_string(FromJointToMotorPosition(act_opt_.max_position_slip,true)) + "\n");
            c_->DiagnosticWrite("conf set servo.max_velocity_slip " + std::to_string(FromJointToMotorPosition(act_opt_.max_velocity_slip,true)) + "\n");
            c_->DiagnosticWrite("conf set servo.enable_motor_temperature " + std::to_string(act_opt_.enable_motor_temperature) + "\n");
            c_->DiagnosticWrite("conf set servo.position_min " + std::to_string(FromJointToMotorPosition(act_opt_.pos_min_limit,true) + position_offset_) + "\n");
            c_->DiagnosticWrite("conf set servo.position_max " + std::to_string(FromJointToMotorPosition(act_opt_.pos_max_limit,true) + position_offset_) + "\n");
            c_->DiagnosticWrite("conf set servo.max_velocity " + std::to_string(FromJointToMotorPosition(act_opt_.max_velocity,true)) + "\n");
            c_->DiagnosticWrite("conf set servo.max_voltage " + std::to_string(act_opt_.max_voltage) + "\n");
            c_->DiagnosticWrite("conf set servo.max_power " + std::to_string(act_opt_.max_power_W) + "\n");
            c_->DiagnosticWrite("conf set servo.max_current " + std::to_string(act_opt_.max_current_A) + "\n");
            c_->DiagnosticWrite("conf set servo.flux_brake_margin " + std::to_string(act_opt_.flux_brake_margin_voltage) + "\n");

            c_->DiagnosticWrite("d exact 0.0\n");
            c_->DiagnosticFlush();
            
            
            return true;
        };

        void Actuator_Manager::ExportSttInt(std::vector<hardware_interface::StateInterface> &stt_int)
        {

            if(query_format_.position != Resolution::kIgnore)
            {
                    stt_int.emplace_back(
                        jnt_name_,
                        hardware_interface::HW_IF_POSITION,
                        &stt_.position
                    );
            }
                if(query_format_.velocity != Resolution::kIgnore)
            {
                    stt_int.emplace_back(
                        jnt_name_,
                        hardware_interface::HW_IF_VELOCITY,
                        &stt_.velocity
                    );
            }
                if(query_format_.torque != Resolution::kIgnore)
            {
                    stt_int.emplace_back(
                        jnt_name_,
                        hardware_interface::HW_IF_EFFORT,
                        &stt_.effort
                    );
            }
            if(query_format_.q_current != Resolution::kIgnore)
            {
                    stt_int.emplace_back(
                        jnt_name_,
                        hardware_interface::HW_IF_Q_CURRENT,
                        &stt_.q_current
                    );
            }
            if(query_format_.d_current != Resolution::kIgnore)
            {
                    stt_int.emplace_back(
                        jnt_name_,
                        hardware_interface::HW_IF_D_CURRENT,
                        &stt_.d_current
                    );
            }
            if(query_format_.abs_position != Resolution::kIgnore)
            {
                    stt_int.emplace_back(
                        jnt_name_,
                        hardware_interface::HW_IF_ABS_POSITION,
                        &stt_.abs_position
                    );
            }
            if(query_format_.power != Resolution::kIgnore)
            {
                    stt_int.emplace_back(
                        jnt_name_,
                        hardware_interface::HW_IF_POWER,
                        &stt_.power
                    );
            }
            if(query_format_.motor_temperature != Resolution::kIgnore)
            {
                    stt_int.emplace_back(
                        jnt_name_,
                        hardware_interface::HW_IF_MOTOR_TEMPERATURE,
                        &stt_.motor_temperature
                    );
            }
            if(query_format_.voltage != Resolution::kIgnore)
            {
                    stt_int.emplace_back(       
                        jnt_name_,
                        hardware_interface::HW_IF_VOLTAGE,
                        &stt_.voltage
                    );
            }
            if(query_format_.temperature != Resolution::kIgnore)
            {
                    stt_int.emplace_back(
                        jnt_name_,
                        hardware_interface::HW_IF_TEMPERATURE,
                        &stt_.temperature
                    );
            }
            for(int i =0; i< MAX_EXTRAS; i++)
            {
                if(query_format_.extra[i].resolution != Resolution::kIgnore)
                {
                    if(query_format_.extra[i].register_number == mjbots::moteus::Register::kControlPositionError)
                    {
                        stt_int.emplace_back(
                            jnt_name_ + "_error",
                            hardware_interface::HW_IF_POSITION,
                            &stt_.position_error
                        );
                    }
                    else if(query_format_.extra[i].register_number == mjbots::moteus::Register::kControlVelocityError)
                    {
                        stt_int.emplace_back(
                            jnt_name_ + "_error",
                            hardware_interface::HW_IF_VELOCITY,
                            &stt_.velocity_error
                        );
                    }
                    else if(query_format_.extra[i].register_number == mjbots::moteus::Register::kControlTorqueError)
                    {
                        stt_int.emplace_back(
                            jnt_name_ + "_error",
                            hardware_interface::HW_IF_EFFORT,
                            &stt_.torque_error
                        );
                    }
                    else if(
                            query_format_.extra[i].register_number == mjbots::moteus::Register::kEncoder0Position || 
                            query_format_.extra[i].register_number == mjbots::moteus::Register::kEncoder1Position || 
                            query_format_.extra[i].register_number == mjbots::moteus::Register::kEncoder2Position 
                        )
                    {
                        stt_int.emplace_back(
                            jnt_name_ + "_second_encoder",
                            hardware_interface::HW_IF_POSITION,
                            &stt_.second_encoder_position
                        );
                    }
                    else if(
                            query_format_.extra[i].register_number == mjbots::moteus::Register::kEncoder0Position || 
                            query_format_.extra[i].register_number == mjbots::moteus::Register::kEncoder1Position || 
                            query_format_.extra[i].register_number == mjbots::moteus::Register::kEncoder2Position
                    )
                    {
                        stt_int.emplace_back(
                            jnt_name_ + "_second_encoder",
                            hardware_interface::HW_IF_VELOCITY,
                            &stt_.second_encoder_velocity
                        );
                    }
                    
                }
            }
        };

         void Actuator_Manager::ExportCmdInt(std::vector<hardware_interface::CommandInterface> &cmd_int)
        {
            cmd_int.emplace_back(
                jnt_name_,
                hardware_interface::HW_IF_POSITION,
                &cmd_.position
            );
            cmd_int.emplace_back(
                jnt_name_,
                hardware_interface::HW_IF_VELOCITY,
                &cmd_.velocity
            );
            cmd_int.emplace_back(
                jnt_name_,
                hardware_interface::HW_IF_EFFORT,
                &cmd_.effort
            );
            cmd_int.emplace_back(
                jnt_name_,
                hardware_interface::HW_IF_KP_SCALE,
                &cmd_.kp_scale
            );
            cmd_int.emplace_back(
                jnt_name_,
                hardware_interface::HW_IF_KD_SCALE,
                &cmd_.kd_scale
            );
        };
        void Actuator_Manager::ParseSttFromReply(CanFdFrame frame)
        {
            mjbots::moteus::Query::Result result = mjbots::moteus::Query::Parse(frame.data, frame.size);
            if(query_format_.position != Resolution::kIgnore)
                stt_.position = FromMotorToJointPosition(result.position) - position_offset_;
            if(query_format_.velocity != Resolution::kIgnore)
                stt_.velocity = FromMotorToJointPosition(result.velocity);
            if(query_format_.torque != Resolution::kIgnore)
                stt_.effort = FromMotorToJointEffort(result.torque);
            if(query_format_.q_current != Resolution::kIgnore)
                stt_.q_current = result.q_current;
            if(query_format_.d_current != Resolution::kIgnore)
                stt_.d_current = result.d_current;
            if(query_format_.abs_position != Resolution::kIgnore)
                stt_.abs_position = FromMotorToJointPosition(result.abs_position);
            if(query_format_.power != Resolution::kIgnore)
                stt_.power = result.power;
            if(query_format_.motor_temperature != Resolution::kIgnore)
                stt_.motor_temperature = result.motor_temperature;
            if(query_format_.voltage != Resolution::kIgnore)
                stt_.voltage = result.voltage;
            if(query_format_.temperature != Resolution::kIgnore)
                stt_.temperature = result.temperature;
            for(int i =0; i< MAX_EXTRAS; i++)
            {

                if(query_format_.extra[i].resolution != Resolution::kIgnore)
                {
                    if(query_format_.extra[i].register_number == mjbots::moteus::Register::kControlPositionError)
                    {
                        stt_.position_error = FromMotorToJointPosition(result.extra[i].value);
                    }
                    else if(query_format_.extra[i].register_number == mjbots::moteus::Register::kControlVelocityError)
                    {
                        stt_.velocity_error = FromMotorToJointPosition(result.extra[i].value);
                    }
                    else if(query_format_.extra[i].register_number == mjbots::moteus::Register::kControlTorqueError)
                    {
                        stt_.torque_error = FromMotorToJointEffort(result.extra[i].value);
                    }
                    else if(
                            (query_format_.extra[i].register_number == mjbots::moteus::Register::kEncoder0Position || 
                            query_format_.extra[i].register_number == mjbots::moteus::Register::kEncoder1Position || 
                            query_format_.extra[i].register_number == mjbots::moteus::Register::kEncoder2Position ) &&
                            second_encoder_output_ != nullptr
                        )
                    {
                        RCLCPP_INFO(rclcpp::get_logger("Actuator_Manager"),"Parsing second encoder position for actuator id %d on bus %d has value %f",id_,bus_,result.extra[i].value);
                        stt_.second_encoder_position = second_encoder_output_->FromeEncoderToJointPosition(result.extra[i].value);
                    }
                    else if(
                            (query_format_.extra[i].register_number == mjbots::moteus::Register::kEncoder0Velocity || 
                            query_format_.extra[i].register_number == mjbots::moteus::Register::kEncoder1Velocity || 
                            query_format_.extra[i].register_number == mjbots::moteus::Register::kEncoder2Velocity) &&
                            second_encoder_output_ != nullptr
                    )
                    {
                        
                        stt_.second_encoder_velocity = second_encoder_output_->FromEncoderToJointVelocity(result.extra[i].value);
                    }
                    else
                    {
                        RCLCPP_WARN(rclcpp::get_logger("Actuator_Manager"),"Extra register number %d not handled for actuator id %d on bus %d",query_format_.extra[i].register_number,id_,bus_);
                    }

                }
            }
        };
        void Actuator_Manager::MakeCommand()
        {
            mjbots::moteus::PositionMode::Command cmd;
            cmd.position = FromJointToMotorPosition(cmd_.position) + position_offset_;
            cmd.velocity = FromJointToMotorPosition(cmd_.velocity);
            cmd.feedforward_torque = FromJointToMotorEffort(Saturation(cmd_.effort,max_torque_));
            cmd.kp_scale = FromJointToMotorGain(cmd_.kp_scale);
            cmd.kd_scale = FromJointToMotorGain(cmd_.kd_scale);
            *cmd_frame_ = c_->MakePosition(cmd);

        };
        void Actuator_Manager::MakeStop()
        {
           *cmd_frame_ = c_->MakeStop();
        };
        

    }
}