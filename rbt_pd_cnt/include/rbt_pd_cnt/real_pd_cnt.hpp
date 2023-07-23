#ifndef REAL_PD_CNT_HPP
#define REAL_PD_CNT_HPP

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "controller_interface/controller_interface.hpp"

#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"

#include "realtime_tools/realtime_buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include <map>
namespace rbt_pd_cnt
{
    using CmdType = sensor_msgs::msg::JointState;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class Real_PD_Cnt : public controller_interface::ControllerInterface
    {
        public:
            Real_PD_Cnt();

            // controller methods

            CallbackReturn on_init() override;
        
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override; 

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(
                const rclcpp::Time & time, const rclcpp::Duration & period
            ) override;
        protected:
            // subscriber 
            rclcpp::Subscription<CmdType>::SharedPtr jnt_cmd_sub_;
            // logger name
            std::string logger_name_;
            // controller parameter
            
            //joint command and state variable
            std::vector<double> init_pos_;
            sensor_msgs::msg::JointState jnt_cmd_,jnt_stt_;

            std::map<std::string,double> position_map_;
            std::map<std::string,double> velocity_map_;
            std::vector<std::string > joint_;
            
          
            bool first_time_;

            //real time buffer
            realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;


    };

};


#endif