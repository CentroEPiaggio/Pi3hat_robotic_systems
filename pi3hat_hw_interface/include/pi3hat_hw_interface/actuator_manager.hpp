#ifndef MOTOR_MANAGER_HPP
#define MOTOR_MANAGER_HPP
#include "pi3hat_hw_interface/elem_info_parsers.hpp"
// mjbots include 
#include "moteus_pi3hat/moteus_protocol.h"
#include "moteus_pi3hat/realtime.h"
#include "moteus_pi3hat/moteus.h"
//ROS2 control import 
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
//std library import 
#include <string>
#include <cstdint>
#include <vector>
#include <functional>
#include <tuple>
#include <memory>
#include <array>
#include <cmath>

#define DELTA 0.5


namespace hardware_interface
{
    constexpr char HW_IF_KP_SCALE[] = "kp_scale_value";
    constexpr char HW_IF_KD_SCALE[] = "kd_scale_value";
    
    constexpr char HW_IF_TEMPERATURE[] = "temperature";
    constexpr char HW_IF_MOTOR_TEMPERATURE[] = "motor_temperature";
    
    
    constexpr char HW_IF_Q_CURRENT[] = "q_current";
    constexpr char HW_IF_D_CURRENT[] = "d_current";
    constexpr char HW_IF_ABS_POSITION[] = "abs_position";
    constexpr char HW_IF_POWER[] = "power";
    constexpr char HW_IF_VOLTAGE[] = "voltage";



    constexpr char HW_IF_VALIDITY_LOSS[] = "validity_loss";
    constexpr char HW_IF_PACKAGE_LOSS[] = "package_loss";
    constexpr char HW_IF_CYCLE_DUR[] = "cycle_duration";

}
namespace pi3hat_hw_interface
{
    namespace actuator_manager
    {
        
        using Controller = mjbots::moteus::Controller;
        using QueryFormat = mjbots::moteus::Query::Format;
        using CanFdFrame = mjbots::moteus::CanFdFrame;
        
        struct CommandStruct
        {
            double position = 0.0;
            double velocity = 0.0;
            double effort = 0.0;
            double kp_scale = 1.0;
            double kd_scale = 1.0;
        };
        struct StateStruct
        {
            double position = 0.0;
            double velocity = 0.0;
            double effort = 0.0;
            double q_current = 0.0;
            double d_current = 0.0;
            double abs_position = 0.0;
            double power = 0.0;
            double motor_temperature = 0.0;
            double temperature = 0.0;
            double voltage = 0.0;
            double second_encoder_position = 0.0;
            double second_encoder_velocity = 0.0;
            double position_error = 0.0;
            double velocity_error = 0.0;
            double torque_error = 0.0;
        };
        
        class SecondEncoderOutput
        {
            public:
                SecondEncoderOutput(double second_encoder_transmission):
                second_encoder_transmission_(second_encoder_transmission)
                {
                    if(second_encoder_transmission_ < 0.0)
                        throw std::runtime_error("Second encoder transmission must be greater than zero");
                };
                ~SecondEncoderOutput()
                {

                };
                double FromeEncoderToJointPosition(double val)
                {
                    double pos, diff;
                    if(first_read)
                    {
                        pos_offset_ = val;
                        first_read = false;
                    }
                    pos = (val - pos_offset_);
                    diff = pos - old_encoder_pos;
                    if(std::abs(diff) > DELTA && diff > 0)
                    {
                        //overflow
                        counter_--;
                    }
                    else if(std::abs(diff) > DELTA && diff < 0)
                    {
                        //underflow
                        counter_++;
                    }
                    old_encoder_pos = pos;
                    return (pos + counter_ * 2 * M_PI) / second_encoder_transmission_;
                };
                double FromEncoderToJointVelocity(double val)
                {
                    return (val) / second_encoder_transmission_;
                };
            private:
                double second_encoder_transmission_;
                bool first_read = true;
                double pos_offset_ = 0.0;
                int counter_ = 0;
                double old_encoder_pos = 0.0;
        };
        class Actuator_Manager
        {
            public:
                Actuator_Manager(CanFdFrame *command_frame)
                {
                    cmd_frame_ = command_frame;
                    
                };
                ~Actuator_Manager();
                void SetActuatorParam(
                    const ActuatorOptions opt,
                    uint16_t id,
                    uint16_t bus,
                    std::string jnt_name
                )
                {
                    id_ = id;
                    bus_ = bus;
                    jnt_name_ = jnt_name;
                    act_opt_ = opt;
                }
                
                void setQueryFormat(const ActuatorQuery query_format)
                {
                    int extra_count = 0;
                    query_format_.position = parse_res(query_format.position);
                    query_format_.velocity = parse_res(query_format.velocity);
                    query_format_.torque = parse_res(query_format.torque);
                    query_format_.d_current = parse_res(query_format.d_current);
                    query_format_.q_current = parse_res(query_format.q_current);
                    query_format_.power = parse_res(query_format.power);
                    query_format_.abs_position = parse_res(query_format.abs_position);
                    query_format_.motor_temperature = parse_res(query_format.motor_temperature);
                    query_format_.temperature = parse_res(query_format.temperature);
                    query_format_.voltage = parse_res(query_format.voltage);
                    if(query_format.position_error != 0)
                    {
                        query_format_.extra[extra_count].resolution = parse_res(query_format.position_error);
                        query_format_.extra[extra_count].register_number = mjbots::moteus::kControlPositionError;
                        extra_count ++;
                    }
                    if(query_format.velocity_error != 0)
                    {
                        query_format_.extra[extra_count].resolution = parse_res(query_format.velocity_error);
                        query_format_.extra[extra_count].register_number = mjbots::moteus::kControlVelocityError;
                        extra_count ++;
                    }
                    if(query_format.torque_error != 0)
                    {
                        query_format_.extra[extra_count].resolution = parse_res(query_format.torque_error);
                        query_format_.extra[extra_count].register_number = mjbots::moteus::kControlTorqueError;
                        extra_count ++;
                    }
                    if(query_format.second_encoder_position != 0 && se_source_ != -1)
                    {
                        query_format_.extra[extra_count].resolution = parse_res(query_format.second_encoder_position);
                        if(se_source_ == 0)
                            query_format_.extra[extra_count].register_number = mjbots::moteus::kEncoder0Position;
                        else if(se_source_ == 1)
                            query_format_.extra[extra_count].register_number = mjbots::moteus::kEncoder1Position;
                        else if(se_source_ == 2)
                            query_format_.extra[extra_count].register_number = mjbots::moteus::kEncoder2Position;
                        else
                            throw std::runtime_error("se source is set wrongly");
                        extra_count ++;
                    }
                     if(query_format.second_encoder_velocity != 0 && se_source_ != -1)
                    {
                        query_format_.extra[extra_count].resolution = parse_res(query_format.second_encoder_velocity);
                        if(se_source_ == 0)
                            query_format_.extra[extra_count].register_number = mjbots::moteus::kEncoder0Velocity;
                        else if(se_source_ == 1)
                            query_format_.extra[extra_count].register_number = mjbots::moteus::kEncoder1Velocity;
                        else if(se_source_ == 2)
                            query_format_.extra[extra_count].register_number = mjbots::moteus::kEncoder2Velocity;
                        else
                            throw std::runtime_error("se source is set wrongly");
                        extra_count ++;
                    }
                    

                    // RCLCPP_INFO(rclcpp::get_logger("Actuator_Manager"),"Set Query Format extra 0 %d %d",query_format_.extra[0].register_number,query_format_.extra[0].resolution);
                    // query_format_.extra[0].register_number = mjbots::moteus::Register::kControlPositionError;
                    // query_format_.extra[0].resolution = mjbots::moteus::Resolution::kFloat;
                    // RCLCPP_INFO(rclcpp::get_logger("Actuator_Manager"),"Set Query Format extra 0 %d %d",query_format_.extra[0].register_number,query_format_.extra[0].resolution);
                };

                void setSecondEncoderSource(unsigned int se_source)
                {
                    se_source_ = se_source;
                };
                std::string get_joint_name()
                {
                    return jnt_name_;
                };
                u_int16_t GetActuatorId()
                {
                    return id_;
                };
                bool ConfigureActuator(std::shared_ptr<mjbots::moteus::Transport> transport);
                
                void ExportSttInt(std::vector<hardware_interface::StateInterface> &stt_int);
                void ExportCmdInt(std::vector<hardware_interface::CommandInterface> &cmd_int);
                void ParseSttFromReply(CanFdFrame frame);
                void MakeCommand();
                void MakeQuery()
                {
                    *cmd_frame_ = c_->MakeQuery(&query_format_);
                }
                void MakeStop();
            private:
                mjbots::moteus::Resolution parse_res(int res)
                {
                    if(res == 0)
                        return mjbots::moteus::Resolution::kIgnore;
                    else if(res == 8)
                        return mjbots::moteus::Resolution::kInt8;
                    else if(res == 16)
                        return mjbots::moteus::Resolution::kInt16;
                    else if(res == 32)
                        return mjbots::moteus::Resolution::kInt32;
                    else if(res == 64)
                        return mjbots::moteus::Resolution::kFloat;
                    else
                        throw std::runtime_error("Wrong resolution format are available just [0,8,16,32,64]");
                }
                double Saturation(double val, double limit)
                {
                    if(val > limit)
                        return limit;
                    else if(val < -limit)
                        return -limit;
                    else
                        return val;
                };
                
                double FromJointToMotorGain(double val, bool saturate = false)
                {
                    double res = (2 * M_PI * val) / std::pow(actuator_transmission_,2);
                    if(saturate)
                        res = Saturation(res,std::pow(2,16));
                    return res;
                   
                };
                
                double FromJointToMotorPosition(double val, bool saturate = false)
                {
                    double res = (val * actuator_transmission_) / (2 * M_PI);
                    if(saturate)
                        res = Saturation(res,std::pow(2,16));
                    return res;
                };

                double FromJointToMotorEffort(double val, bool saturate = false)
                {
                    double res = val / actuator_transmission_;
                    if(saturate)
                        res = Saturation(res,std::pow(2,16));
                    return res;
                };
                double FromMotorToJointPosition(double val)
                {
                    return (val * 2 * M_PI) / actuator_transmission_;
                };
                double FromMotorToJointEffort(double val)
                {
                    return val * actuator_transmission_;
                };

                std::unique_ptr<Controller> c_;
                ActuatorOptions act_opt_;
                QueryFormat query_format_;
                int se_source_;
                CanFdFrame* cmd_frame_;
                std::string jnt_name_;
                u_int16_t id_,bus_;
                double max_torque_ = 0.0;
                double position_offset_ = 0.0;
                double actuator_transmission_ = 1.0;
                double second_encoder_transmission_ = 0.0;
                CommandStruct cmd_;
                StateStruct stt_;
                std::unique_ptr<SecondEncoderOutput> second_encoder_output_ = nullptr;

                
            };
    };
};

#endif