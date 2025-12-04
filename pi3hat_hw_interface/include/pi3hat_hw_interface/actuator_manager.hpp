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
                Actuator_Manager(CanFdFrame *command_frame, ActuatorOptions options)
                {
                    cmd_frame_ = command_frame;
                    act_opt_ = options;
                    if(act_opt_.second_encoder_trasmission > 0.0)
                    {
                        second_encoder_output_ = std::make_unique<SecondEncoderOutput>(act_opt_.second_encoder_trasmission);
                    }
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
                bool ConfigureActuator(const QueryFormat query_format, std::shared_ptr<mjbots::moteus::Transport> transport, int second_encoder_source);
                
                void ExportSttInt(std::vector<hardware_interface::StateInterface> &stt_int);
                void ExportCmdInt(std::vector<hardware_interface::CommandInterface> &cmd_int);
                void ParseSttFromReply(CanFdFrame frame);
                void MakeCommand();
                void MakeStop();
            private:
                
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

                CanFdFrame* cmd_frame_;
                std::string jnt_name_;
                u_int16_t id_,bus_;
                double max_torque_ = 0.0;
                double position_offset_ = 0.0;
                double actuator_transmission_ = 1.0;
                double second_encoder_transmission_ = 0.0;
               
                CommandStruct cmd_;
                StateStruct stt_;
                std::unique_ptr<SecondEncoderOutput> second_encoder_output_;
                
            };
    };
};

#endif