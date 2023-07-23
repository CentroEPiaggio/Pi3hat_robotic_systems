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
#include <future>
#include <functional>
#include <stdio.h>


#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"
#include "hardware_interface/handle.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "pi3hat_hw_interface/motor_manager.hpp"

#define NUM_STOP 10


using namespace mjbots;
using pi3hat_hw_interface::motor_manager::Motor_Manager;
using MoteusInterface = moteus::Pi3HatMoteusInterface;
using Command = moteus::Pi3HatMoteusInterface::ServoCommand;
using Reply = moteus::Pi3HatMoteusInterface::ServoReply;
using Options = moteus::Pi3HatMoteusInterface::Options;
using Data = moteus::Pi3HatMoteusInterface::Data;
using Output = moteus::Pi3HatMoteusInterface::Output;
using namespace std::chrono_literals;

#define SLEEP_FOR_10MS 10000000ns

using Get_Function = std::function<moteus::QueryResultV2 ( std::vector<Reply>& replies, int bus, int id, int opt,int& err,int provided_msg)>;
using Policy_Function = std::function<void( bool msg_valid, bool msg_coplete, Command* cmd_d)>;

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

                void cycle()
                {
                    auto promise = make_shared<std::promise<Output>>();
                    communication_thread_.Cycle(
                            data_,
                            [promise](const Output& out)
                            {
                                promise->set_value(out);
                            }
                        );
                    can_recvd_ = promise->get_future();
                    //RCLCPP_INFO(rclcpp::get_logger("LOGGER_NAME"),"Cycle Call valid %d", can_recvd_.valid());
                    // RCLCPP_INFO(rclcpp::get_logger("LOGGER_NAME"),"Cycle Call gets %d", can_recvd_.get().query_result_size);

                };
            private:
                
                std::vector<Motor_Manager> motors_;
                std::vector<Command> cmd_data_;
                std::vector<Reply> msr_data_;
                Options opt_;
                MoteusInterface communication_thread_;
                Get_Function gets_;
                Policy_Function poly_;
                Data data_;
                Output out_;
                Options opt_thread_;
                std::future<Output> can_recvd_;

        };
    }
}



#endif