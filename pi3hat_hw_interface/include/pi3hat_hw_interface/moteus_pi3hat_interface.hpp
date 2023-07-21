#ifndef MOTEUS_PI3HAT_HW_INT_HPP
#define MOTEUS_PI3HAT_HW_INT_HPP
#include "hardware_interface/system.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pi3hat_hw_interface/motor_manager.hpp"


#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <chrono>

#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"
#include "hardware_interface/handle.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

using namespace mjbots;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace pi3hat_hw_interface
{
    namespace moteus_pi3hat_interface
    {
        class MoteusPi3Hat_Interface : public hardware_interface::SystemInterface
        {
            public:
                MoteusPi3Hat_Interface();
                ~MoteusPi3Hat_Interface();
                CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
                CallbackReturn on_configure(const rclcpp_lifecycle::State& ) override;
                CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

                std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
                std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

                hardware_interface::return_type read(const rclcpp::Time & , const rclcpp::Duration & ) override;
                hardware_interface::return_type write(const rclcpp::Time & , const rclcpp::Duration & ) override;
            private:
             std::vector<int> a_;
             std::pair<int,int> b_;
             std::vector<std::pair<int,int>> c_;
        };
    }
}



#endif