#include "pluginlib/class_list_macros.hpp"
#include "pi3hat_base_controller/moteus_debug.hpp"
#include "pi3hat_hw_interface/actuator_manager.hpp"
#include <cstdint>
#include <cmath>

namespace debug_broadcaster
{
    Debug_Broadcaster::Debug_Broadcaster():
    logger_name_("Debug_Broadcaster"),
    stt_pub_(nullptr)
    {};

    CallbackReturn Debug_Broadcaster::on_init()
    {
        
         try
        {
            auto_declare<std::vector<std::string>>("joints",std::vector<std::string>());
        }
         catch(const std::exception & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Exception thrown during declaration of joints name with message: %s", e.what());
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_node()->get_logger(),"initialize succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Debug_Broadcaster::on_configure(const rclcpp_lifecycle::State &)
    {
        
        joints_ = get_node()->get_parameter("joints").as_string_array();
        if(joints_.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'joints' parameter is empty");
            return CallbackReturn::ERROR;
        }
        

        stt_msg_.name.resize(joints_.size());
        stt_msg_.position.resize(joints_.size());
        stt_msg_.velocity.resize(joints_.size());
        stt_msg_.effort.resize(joints_.size());
        stt_msg_.temperature.resize(joints_.size());
        stt_msg_.current.resize(joints_.size());
  
        stt_pub_ = get_node()->create_publisher<StateMsgs>(
            "~/joints_state",
            5
        );
        RCLCPP_INFO(get_node()->get_logger(),"configurated succesfully");
        return CallbackReturn::SUCCESS;

    }

    CallbackReturn Debug_Broadcaster::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Debug_Broadcaster::on_activate(const rclcpp_lifecycle::State &)
    {
        // for(size_t i = 0; i < joints_.size(); i++)
        //     RCLCPP_INFO(get_node()->get_logger(),"joints %d has name %s and is %s",i,joints_[i].c_str(),se_flag_[i]?"active":"inactive");

        RCLCPP_INFO(get_node()->get_logger(),"activated succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Debug_Broadcaster::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration Debug_Broadcaster::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration stt_int_cnf;
        stt_int_cnf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        
        for(size_t i = 0; i < joints_.size(); i++)
        {
            stt_int_cnf.names.push_back(joints_[i] + "_error/" + hardware_interface::HW_IF_POSITION);
            stt_int_cnf.names.push_back(joints_[i] + "_error/" + hardware_interface::HW_IF_VELOCITY);
            stt_int_cnf.names.push_back(joints_[i] + "_error/" + hardware_interface::HW_IF_EFFORT);
        
        }
        // RCLCPP_INFO(get_node()->get_logger(),"the dimesion of hw is %ld",stt_int_cnf.names.size());
        return stt_int_cnf;
    }

    controller_interface::InterfaceConfiguration Debug_Broadcaster::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration cmd_int_cnf;
        cmd_int_cnf.type = controller_interface::interface_configuration_type::NONE;
        return cmd_int_cnf;
    }

    controller_interface::return_type Debug_Broadcaster::update(const rclcpp::Time & time, const rclcpp::Duration &)
    {
        size_t sz = joints_.size();


        stt_msg_.header.set__stamp(time);
        for(size_t i = 0; i < sz; i++)
        {
            // RCLCPP_INFO(get_node()->get_logger(),"executing std stt for jnt %s",joints_[i].c_str());
            stt_msg_.name[i] = joints_[i];
            // RCLCPP_INFO(get_node()->get_logger(),"temp ind is %ld",1 + sz + 4*i );
            stt_msg_.position[i] = state_interfaces_[3*i ].get_value();
            // RCLCPP_INFO(get_node()->get_logger(),"temp ind is %ld",1 + sz + 4*i + 1);
            stt_msg_.velocity[i] = state_interfaces_[3*i+ 1].get_value();
            // RCLCPP_INFO(get_node()->get_logger(),"temp ind is %ld",1 + sz + 4*i + 2);
            stt_msg_.effort[i] = state_interfaces_[3*i + 2].get_value();
            // RCLCPP_INFO(get_node()->get_logger(),"temp ind is %ld",1 + sz + 4*i + 3);
           
            
        }
    
        
        stt_pub_->publish(stt_msg_);

        return controller_interface::return_type::OK;

    }
};

PLUGINLIB_EXPORT_CLASS(
    debug_broadcaster::Debug_Broadcaster, controller_interface::ControllerInterface
);