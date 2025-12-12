#ifndef PI3HAT_JOINT_GROUP_CONTROLLER_HPP
#define PI3HAT_JOINT_GROUP_CONTROLLER_HPP
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "pi3hat_moteus_int_msgs/msg/joints_command.hpp"
#include "realtime_tools/realtime_buffer.h"

namespace pi3hat_joint_group_controller
{
    using CmdMsgs = pi3hat_moteus_int_msgs::msg::JointsCommand;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class Pi3Hat_Joint_Group_Controller : public controller_interface::ControllerInterface
    {
        public:
            Pi3Hat_Joint_Group_Controller();

            ~Pi3Hat_Joint_Group_Controller();
            
            CallbackReturn on_init() override;
            
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override; 

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(
                const rclcpp::Time & time, const rclcpp::Duration & period
            ) override;

            bool get_reference();
        
        private:
            rclcpp::Subscription<CmdMsgs>::SharedPtr cmd_sub_;
            std::string logger_name_;
            std::map<std::string,double> position_cmd_, velocity_cmd_,effort_cmd_,kp_scale_cmd_,kd_scale_cmd_;
            
            realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsgs>> rt_buffer_;
            
            std::shared_ptr<CmdMsgs> joints_rcvd_msg_;
            bool default_init_pos_;
    };
};


#endif