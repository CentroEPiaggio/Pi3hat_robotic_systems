#include "pluginlib/class_list_macros.hpp"
#include "pi3hat_base_controller/pi3hat_joint_group_controller.hpp"
#include "pi3hat_hw_interface/motor_manager.hpp"    
#include <cstdint>

namespace pi3hat_joint_group_controller
{
    Pi3Hat_Joint_Group_Controller::Pi3Hat_Joint_Group_Controller():
    cmd_sub_(nullptr),
    logger_name_("Pi3Hat_Joint_Group_Controller"),
    rt_buffer_(nullptr),
    joints_rcvd_msg_(nullptr),
    default_init_pos_(false)
    
    {}

    Pi3Hat_Joint_Group_Controller::~Pi3Hat_Joint_Group_Controller()
    {
        // rt_buffer_.~RealtimeBuffer();
    }

    CallbackReturn Pi3Hat_Joint_Group_Controller::on_init()
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
        try
        {
            auto_declare<std::vector<double>>("init_pos",std::vector<double>());
        }
          catch(const std::exception & e)
        {
            RCLCPP_WARN(rclcpp::get_logger(logger_name_),"Exception thrown during declaretion of init position with message: %s it gets default values", e.what());
            
        }
        default_init_pos_ = true;
        joints_rcvd_msg_ = std::make_shared<CmdMsgs>();
        RCLCPP_INFO(get_node()->get_logger(),"initialize succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Joint_Group_Controller::on_configure(const rclcpp_lifecycle::State &)
    {
        std::vector<double> init_positions;
        size_t sz;
        // get the controlled joints name
        std::vector<std::string> joint = get_node()->get_parameter("joints").as_string_array();
        if(joint.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'joints' parameter is empty");
            return CallbackReturn::ERROR;
        }
        
        // init the initial position if its needed
        if(!default_init_pos_)
        {
            init_positions = get_node()->get_parameter("init_pos").as_double_array();
            if(init_positions.empty())
            {
                RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'init_pos' parameter is empty");
                return CallbackReturn::ERROR;
            }
            if(init_positions.size() != joint.size())
            {
                RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'init_pos' and 'joints' can not have different dimension");
                return CallbackReturn::ERROR;
            }
        }
        else
            init_positions.resize(joint.size(),0.0);
        
        // fill the map structure 
        sz = joint.size();
        for(size_t i = 0; i < sz; i++)
        {
            position_cmd_.emplace(std::make_pair(joint[i],init_positions[i]));
            velocity_cmd_.emplace(std::make_pair(joint[i],0.0));
            effort_cmd_.emplace(std::make_pair(joint[i],0.0));
            kp_scale_cmd_.emplace(std::make_pair(joint[i],1.0));
            kd_scale_cmd_.emplace(std::make_pair(joint[i],1.0));

        }

        // build the subscriber
        
        cmd_sub_ = get_node()->create_subscription<CmdMsgs>(
            "~/command",
            5,
            [this](const CmdMsgs::SharedPtr msg)
            {
                // rt_buffer_.writeFromNonRT(msg);
                joints_rcvd_msg_->set__name(msg->name);
                joints_rcvd_msg_->set__position(msg->position);
                joints_rcvd_msg_->set__velocity(msg->velocity);
                joints_rcvd_msg_->set__effort(msg->effort);
                joints_rcvd_msg_->set__kp_scale(msg->kp_scale);
                joints_rcvd_msg_->set__kd_scale(msg->kd_scale);
            }
        );
        RCLCPP_INFO(get_node()->get_logger(),"configure succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Joint_Group_Controller::on_activate(const rclcpp_lifecycle::State &)
    {
        rt_buffer_.reset();
        RCLCPP_INFO(get_node()->get_logger(),"activate succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Joint_Group_Controller::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Joint_Group_Controller::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    bool Pi3Hat_Joint_Group_Controller::get_reference()
    {
        // joints_rcvd_msg_ = *rt_buffer_.readFromRT();

        if(joints_rcvd_msg_.get())
        {
            
            for(size_t i = 0; i < joints_rcvd_msg_->name.size(); i++)
            {
                try
                {   
                    // RCLCPP_INFO(get_node()->get_logger(),"pass name %s",joints_rcvd_msg_->name[i].c_str());
                    if(joints_rcvd_msg_->position.size() == 0 || joints_rcvd_msg_->position.size() != joints_rcvd_msg_->name.size())
                        position_cmd_.at(joints_rcvd_msg_->name[i]) = 0.0;
                    else
		    {
                        position_cmd_.at(joints_rcvd_msg_->name[i]) = joints_rcvd_msg_->position[i];
			// RCLCPP_INFO(get_node()->get_logger(),"the %s joint  pos is %f",joints_rcvd_msg_->name[i].c_str(),joints_rcvd_msg_->position[i]);
		    }
                    if(joints_rcvd_msg_->velocity.size() == 0 || joints_rcvd_msg_->velocity.size() != joints_rcvd_msg_->name.size())
                        velocity_cmd_.at(joints_rcvd_msg_->name[i]) = 0.0;
                    else
                        velocity_cmd_.at(joints_rcvd_msg_->name[i]) = joints_rcvd_msg_->velocity[i];
                    if(joints_rcvd_msg_->effort.size() == 0 || joints_rcvd_msg_->effort.size() != joints_rcvd_msg_->name.size())
                        effort_cmd_.at(joints_rcvd_msg_->name[i]) = 0.0;
                    else
                        effort_cmd_.at(joints_rcvd_msg_->name[i]) = joints_rcvd_msg_->effort[i];
                    if(joints_rcvd_msg_->kp_scale.size() == 0 || joints_rcvd_msg_->kp_scale.size() != joints_rcvd_msg_->name.size())
                        kp_scale_cmd_.at(joints_rcvd_msg_->name[i]) = 0.0;
                    else
                        kp_scale_cmd_.at(joints_rcvd_msg_->name[i]) = joints_rcvd_msg_->kp_scale[i];
                    if(joints_rcvd_msg_->kd_scale.size() == 0 || joints_rcvd_msg_->kd_scale.size() != joints_rcvd_msg_->name.size())
                        kd_scale_cmd_.at(joints_rcvd_msg_->name[i]) = 0.0;
                    else
                        kd_scale_cmd_.at(joints_rcvd_msg_->name[i]) = joints_rcvd_msg_->kd_scale[i];
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR( rclcpp::get_logger(logger_name_),"Raised error during the reference assegnation %s", e.what());
                    return false;
                }
            }
        }
        else
            // RCLCPP_WARN(rclcpp::get_logger(logger_name_),"No data are readed from realtime buffer");
            
        // maybe needed?
        //joints_rcvd_msg_.reset();
        
        return true;
    }
    
    controller_interface::return_type Pi3Hat_Joint_Group_Controller::update(const rclcpp::Time & , const rclcpp::Duration & )
    {
        std::string type;
        // set the data from the readed message
        // if(!get_reference())
        //     return controller_interface::return_type::ERROR;
        get_reference();

        // set the commanded reference iterating over the exported commanded interface
        for(auto &cmd_int : command_interfaces_)
        {
            type = cmd_int.get_interface_name();
            try
            {
                if(type == hardware_interface::HW_IF_POSITION)
                    cmd_int.set_value(position_cmd_.at(cmd_int.get_prefix_name()));
                else if(type == hardware_interface::HW_IF_VELOCITY)
                    cmd_int.set_value(velocity_cmd_.at(cmd_int.get_prefix_name()));
                else if(type == hardware_interface::HW_IF_EFFORT)
                    cmd_int.set_value(effort_cmd_.at(cmd_int.get_prefix_name()));
                else if(type == hardware_interface::HW_IF_KP_SCALE)
                    cmd_int.set_value(kp_scale_cmd_.at(cmd_int.get_prefix_name()));
                else if(type == hardware_interface::HW_IF_KD_SCALE)
                    cmd_int.set_value(kd_scale_cmd_.at(cmd_int.get_prefix_name()));
                else
                {
                    // RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Interface name not correspond to the declared one");
                    return controller_interface::return_type::ERROR;
                }
            }
            catch(std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"catch error %s during the access to '%s' data",e.what(),cmd_int.get_name().c_str());
            }
        }

        //  for(auto &cmd_int : command_interfaces_)
        // {
        //     type = cmd_int.get_interface_name();
        //     try
        //     {
        //         if(type == hardware_interface::HW_IF_POSITION)
        //             RCLCPP_INFO(rclcpp::get_logger("AAA"),"pos is %f",cmd_int.get_value());
                
        //         else
        //         {
        //             RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Interface name not correspond to the declared one");
        //             return controller_interface::return_type::ERROR;
        //         }
        //     }
        //     catch(std::exception &e)
        //     {
        //         RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"catch error %s during the access to '%s' data",e.what(),cmd_int.get_name().c_str());
        //     }
        // }
 
        return controller_interface::return_type::OK;

    }
    
    controller_interface::InterfaceConfiguration Pi3Hat_Joint_Group_Controller::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration stt_int_cnf;
        stt_int_cnf.type = controller_interface::interface_configuration_type::NONE;
        return stt_int_cnf;
    }

    controller_interface::InterfaceConfiguration Pi3Hat_Joint_Group_Controller::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration cmd_int_cnf;
        cmd_int_cnf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(auto &it : position_cmd_)
        {
            cmd_int_cnf.names.push_back(it.first + "/" + hardware_interface::HW_IF_POSITION);
            cmd_int_cnf.names.push_back(it.first + "/" + hardware_interface::HW_IF_VELOCITY);
            cmd_int_cnf.names.push_back(it.first + "/" + hardware_interface::HW_IF_EFFORT);
            cmd_int_cnf.names.push_back(it.first + "/" + hardware_interface::HW_IF_KP_SCALE);
            cmd_int_cnf.names.push_back(it.first + "/" + hardware_interface::HW_IF_KD_SCALE);
        }
        return cmd_int_cnf;
    }
};

PLUGINLIB_EXPORT_CLASS(
    pi3hat_joint_group_controller::Pi3Hat_Joint_Group_Controller, controller_interface::ControllerInterface
);
