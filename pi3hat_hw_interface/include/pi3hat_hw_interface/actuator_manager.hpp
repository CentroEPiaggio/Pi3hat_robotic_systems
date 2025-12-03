#ifndef MOTOR_MANAGER_HPP
#define MOTOR_MANAGER_HPP
#include "moteus_pi3hat/moteus_protocol.h"
#include "moteus_pi3hat/realtime.h"
#include "moteus_pi3hat/pi3hat_moteus_transport.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include <string>
#include <cstdint>
#include <vector>
#include <functional>
#include <tuple>

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
    constexpr char HW_IF_POSITION_ERROR[] = "position_error";
    constexpr char HW_IF_VELOCITY_ERROR[] = "velocity_error";
    constexpr char HW_IF_TORQUE_ERRROR[] = "torque_error";


    constexpr char HW_IF_VALIDITY_LOSS[] = "validity_loss";
    constexpr char HW_IF_PACKAGE_LOSS[] = "package_loss";
    constexpr char HW_IF_CYCLE_DUR[] = "cycle_duration";
    constexpr char HW_IF_W2R_DUR[] = "write2read_duration";

}
namespace pi3hat_hw_interface
{
    namespace actuator_manager
    {
        
        class Actuator_Manager
        {
            public:

            private:
               
        };
    };
};

#endif