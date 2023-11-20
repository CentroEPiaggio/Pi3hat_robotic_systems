#include <cmath>
#include "rbt_pd_cnt/robot_pd_cnt.hpp"

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

    Rbt_PD_cnt::Rbt_PD_cnt():
    controller_interface::ControllerInterface(),
    rt_command_ptr_(nullptr),
    jnt_cmd_sub_(nullptr)
    {
        logger_name_ = "Robot_PD_Controller";
        
    }
    CallbackReturn Rbt_PD_cnt::on_init()
    {
        first_time_ = true;
        try{        
            auto_declare<std::vector<double>>("K_d",std::vector<double>());
            auto_declare<std::vector<double>>("K_p",std::vector<double>());
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
    CallbackReturn Rbt_PD_cnt::on_configure(const rclcpp_lifecycle::State & )
    {
        joint_ = get_node()->get_parameter("joint").as_string_array();
        K_p_ = get_node()->get_parameter("K_p").as_double_array();
        K_d_ = get_node()->get_parameter("K_d").as_double_array();
        if(K_p_.empty() || K_d_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(),"'K_p_' or 'K_d' parameter is empty");
            return CallbackReturn::ERROR;
        }
        if(K_p_.size() != K_d_.size())
        {
            RCLCPP_ERROR(get_node()->get_logger(),"'K_p_' and 'K_d' has different size");
            return CallbackReturn::ERROR; 
        }
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
        for(size_t i = 0; i<K_p_.size();i++)
        {
            if(K_p_[i] < 0 || K_d_[i] < 0)
            {
                RCLCPP_ERROR(get_node()->get_logger(),"PD Gains must be positive");
                return CallbackReturn::ERROR;
            } 
        }
     
        //jnt_pos_stt_.resize(init_pos_.size());
        std::vector<double> zeros(init_pos_.size(),0.0);
        jnt_cmd_.set__position(init_pos_);
        jnt_stt_.set__position(init_pos_);

        jnt_cmd_.set__velocity(zeros);
        jnt_cmd_.set__effort(zeros);
        
        jnt_stt_.set__velocity(zeros);
        jnt_stt_.set__effort(zeros);
        
        jnt_cmd_sub_ = get_node()->create_subscription<CmdType>(
            "~/command", rclcpp::SystemDefaultsQoS(),
            [this](const CmdType::SharedPtr msg){rt_command_ptr_.writeFromNonRT(msg);}
        );
        jnt_stt_pub_ = get_node()->create_publisher<CmdType>("Joint_Feedback",10);

        RCLCPP_INFO(get_node()->get_logger(),"configure succesfull");
        return CallbackReturn::SUCCESS;
    }  

    controller_interface::InterfaceConfiguration Rbt_PD_cnt::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(const auto & joint : joint_)
            command_interface_config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
        return command_interface_config;
    }

    controller_interface::InterfaceConfiguration Rbt_PD_cnt::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(const auto & joint : joint_)
        {
            state_interface_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
            state_interface_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        return state_interface_config;
    }

    CallbackReturn Rbt_PD_cnt::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        //  check if we have all resources defined in the "points" parameter
        //  also verify that we *only* have the resources defined in the "points" parameter
        std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
        std::vector<std::reference_wrapper<LoanedStateInterface>> ordered_stt_interfaces_v,ordered_stt_interfaces_p;
        if (
            !controller_interface::get_ordered_interfaces(
            command_interfaces_, joint_, hardware_interface::HW_IF_EFFORT, ordered_interfaces) ||
            command_interfaces_.size() != ordered_interfaces.size())
        {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Expected %zu effort command interfaces, got %zu", joint_.size(),
            ordered_interfaces.size());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
         if (
            !controller_interface::get_ordered_interfaces(
            state_interfaces_, joint_, hardware_interface::HW_IF_POSITION, ordered_stt_interfaces_p) ||
             !controller_interface::get_ordered_interfaces(
            state_interfaces_, joint_, hardware_interface::HW_IF_VELOCITY, ordered_stt_interfaces_v) ||
            state_interfaces_.size() != ordered_stt_interfaces_p.size() + ordered_stt_interfaces_v.size())
        {
            RCLCPP_ERROR(
            get_node()->get_logger(), "activation error");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        // reset command buffer if a command came through callback when controller was inactive
        rt_command_ptr_.reset();
        RCLCPP_INFO(
            get_node()->get_logger(), "activation succesfull");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Rbt_PD_cnt::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
    {
    // reset command buffer
    rt_command_ptr_.reset();
    return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type Rbt_PD_cnt::update(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
    )
    {
        //read joints position and velocity
        for(uint i = 0; i < jnt_stt_.position.size(); i++)
        {   
            
            jnt_stt_.position[i] = state_interfaces_[2*i].get_value();
            jnt_stt_.velocity[i] = state_interfaces_[2*i +1].get_value();
            std::string pp = state_interfaces_[2*i].get_name();
            if(first_time_)
            {
                RCLCPP_INFO(
                    get_node()->get_logger(),
                    "interface %s has value = %f",pp.c_str(),jnt_stt_.position[i]
                );

            }
            // RCLCPP_INFO(
            //     get_node()->get_logger(),
            //     "joint %d pos has name %s",i,pp.c_str()
            // );
            // RCLCPP_INFO(
            //     get_node()->get_logger(),"Joint %d has pos %f and vel %f",i,jnt_stt_.position[i],jnt_stt_.velocity[i]
            // );
        }
        jnt_stt_.set__name(joint_);
        jnt_stt_pub_->publish(jnt_stt_);
        std::vector<double> zeros(init_pos_.size(),0.0);
        jnt_cmd_.set__effort(zeros);
        auto joint_command = rt_command_ptr_.readFromRT();

        if(!joint_command || !(*joint_command))
        {
          //  RCLCPP_INFO(get_node()->get_logger(),"steady controll");
        }
         else
        {
            jnt_cmd_.set__position((*joint_command)->position);
            jnt_cmd_.set__velocity((*joint_command)->velocity);
            jnt_cmd_.set__effort((*joint_command)->effort);
            // RCLCPP_INFO(get_node()->get_logger(),"arrived new command");
        }
        // // comment if add effort command;
        //     std::vector<double> zeros(jnt_cmd_.effort.size(),0.0);
        //     jnt_cmd_.set__effort(zeros);
        // set effort according to PD policy 
        for(uint i = 0; i < jnt_stt_.position.size(); i++)
        {
            if(first_time_)
            {
                RCLCPP_INFO(
                    get_node()->get_logger(),
                    "joint %d has value = %f and cmd = %f",i,jnt_stt_.position[i],jnt_cmd_.position[i]
                );

            }
            jnt_cmd_.effort[i] = K_p_[i]*(jnt_cmd_.position[i] - jnt_stt_.position[i]) +
                                 K_d_[i]*(jnt_cmd_.velocity[i] - jnt_stt_.velocity[i]) 
                                 + jnt_cmd_.effort[i];
            command_interfaces_[i].set_value(jnt_cmd_.effort[i]);
            // RCLCPP_INFO(
            //         get_node()->get_logger(),
            //         "joint %d has cmd = %f",i,-jnt_cmd_.effort[i]
            //     );
        }
        if(first_time_)
        {
            first_time_ = false;

        }
        return controller_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(
    rbt_pd_cnt::Rbt_PD_cnt, controller_interface::ControllerInterface
)