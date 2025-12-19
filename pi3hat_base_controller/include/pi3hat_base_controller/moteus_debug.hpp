#ifndef DEBUG_BROADCASTER_HPP
#define DEBUG_BROADCASTER_HPP
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/qos.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"
#include "pi3hat_moteus_int_msgs/msg/packet_pass.hpp"


namespace debug_broadcaster
{
    using StateMsgs = pi3hat_moteus_int_msgs::msg::JointsStates;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class Debug_Broadcaster : public controller_interface::ControllerInterface
    {
        public:
            Debug_Broadcaster();

            ~Debug_Broadcaster() {};

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
        private:
            // probably joints_ is useless
            std::vector<std::string> joints_;
            std::vector<bool> se_flag_;
            std::string logger_name_;
            StateMsgs stt_msg_;
            rclcpp::Publisher<StateMsgs>::SharedPtr stt_pub_;

    };
};


#endif