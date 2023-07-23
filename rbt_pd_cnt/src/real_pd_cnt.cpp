#include <cmath>
#include "rbt_pd_cnt/real_pd_cnt.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "controller_interface/helpers.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace rbt_pd_cnt
{
    using hardware_interface::LoanedCommandInterface;
    using hardware_interface::LoanedStateInterface;


    Real_PD_Cnt::Real_PD_Cnt():
    controller_interface::ControllerInterface(),
    rt_command_ptr_(nullptr),
    jnt_cmd_sub_(nullptr)
    {
        logger_name_ = "Real_PD_Controller";
        
    }
    CallbackReturn Real_PD_Cnt::on_init()
    {
        first_time_ = true;
        try{        
            auto_declare<std::vector<std::string>>("joint", std::vector<std::string>());
            auto_declare<std::vector<double>>("init_pos",std::vector<double>());
        }
        catch(const std::exception & e)
        {
            fprintf(stderr,"Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
        
    }
    CallbackReturn Real_PD_Cnt::on_configure(const rclcpp_lifecycle::State & )
    {
        joint_ = get_node()->get_parameter("joint").as_string_array();
        init_pos_ = get_node()->get_parameter("init_pos").as_double_array();

        if(joint_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(),"'joint' parameter is empty");
            return CallbackReturn::ERROR;
        }
        if(init_pos_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(),"'init_pos' parameter is empty");
            return CallbackReturn::ERROR;
        }
        if(joint_.size() != init_pos_.size())
        {
            RCLCPP_ERROR(get_node()->get_logger(),"'start_command' and 'joint' dimensions are different");
            return CallbackReturn::ERROR;
        }
        
        //jnt_pos_stt_.resize(init_pos_.size());
        std::vector<double> zeros(init_pos_.size(),0.0);
        jnt_cmd_.set__position(init_pos_);

        jnt_cmd_.set__velocity(zeros);
        jnt_cmd_.set__effort(zeros);

        for(auto joint: joint_)
        {
            position_map_.emplace(std::make_pair(joint,0.0));
            velocity_map_.emplace(std::make_pair(joint, 0.0));
        }
        
        
        jnt_cmd_sub_ = get_node()->create_subscription<CmdType>(
            "~/command", 5,
            [this](const CmdType::SharedPtr msg){
                for(int i = 0; i < msg->name.size(); i++)
                {
                    position_map_[msg->name[i]] = msg->position[i];
                    velocity_map_[msg->name[i]] = msg->velocity[i];
                }
            }
        );

        RCLCPP_INFO(get_node()->get_logger(),"configure succesfull");
        return CallbackReturn::SUCCESS;
    }  

    controller_interface::InterfaceConfiguration Real_PD_Cnt::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(const auto & joint : joint_)
        {
            command_interface_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
            command_interface_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        return command_interface_config;
    }

    controller_interface::InterfaceConfiguration Real_PD_Cnt::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::NONE;
        return command_interface_config;
    }

    CallbackReturn Real_PD_Cnt::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        //  check if we have all resources defined in the "points" parameter
        //  also verify that we *only* have the resources defined in the "points" parameter
        std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces_p,ordered_interfaces_v;       
         if (
            !controller_interface::get_ordered_interfaces(
            command_interfaces_, joint_, hardware_interface::HW_IF_POSITION, ordered_interfaces_p) ||
             !controller_interface::get_ordered_interfaces(
            command_interfaces_, joint_, hardware_interface::HW_IF_VELOCITY, ordered_interfaces_v) ||
            command_interfaces_.size() != ordered_interfaces_p.size() + ordered_interfaces_v.size())
        {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Expected %zu effort command interfaces, got %zu", 2*(joint_.size()),
            ordered_interfaces_p.size()+ordered_interfaces_v.size());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        // reset command buffer if a command came through callback when controller was inactive
        // rt_command_ptr_.reset();
        RCLCPP_INFO(
            get_node()->get_logger(), "activation succesfull");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Real_PD_Cnt::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
    {
    // reset command buffer
    // rt_command_ptr_.reset();
    return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type Real_PD_Cnt::update(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
    )
    {
        
        
        for(size_t i = 0; i< command_interfaces_.size(); i++)
        {
           
            if( command_interfaces_[i].get_interface_name() == hardware_interface::HW_IF_POSITION)
                command_interfaces_[i].set_value(position_map_[command_interfaces_[i].get_name()]);
            if( command_interfaces_[i].get_interface_name() == hardware_interface::HW_IF_VELOCITY)
                command_interfaces_[i].set_value(velocity_map_[command_interfaces_[i].get_name()]);
        }
        if(first_time_)
        {
            first_time_ = false;

        }
        return controller_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(
    rbt_pd_cnt::Real_PD_Cnt, controller_interface::ControllerInterface
)