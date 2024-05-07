#include "pluginlib/class_list_macros.hpp"
#include "pi3hat_base_controller/pi3hat_state_broadcaster.hpp"
#include "pi3hat_hw_interface/motor_manager.hpp"
#include <cstdint>
#include <cmath>

namespace pi3hat_state_broadcaster
{
    Pi3Hat_State_Broadcaster::Pi3Hat_State_Broadcaster():
    logger_name_("Pi3Hat_State_Broadcaster"),
    per_pub_(nullptr),
    stt_pub_(nullptr)
    {};

    CallbackReturn Pi3Hat_State_Broadcaster::on_init()
    {
        
         try
        {
            auto_declare<std::vector<std::string>>("joints",std::vector<std::string>());
            auto_declare<std::vector<std::string>>("second_encoders",std::vector<std::string>());
            auto_declare<bool>("performance_index",true);
        }
         catch(const std::exception & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Exception thrown during declaration of joints name with message: %s", e.what());
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_node()->get_logger(),"initialize succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_State_Broadcaster::on_configure(const rclcpp_lifecycle::State &)
    {
        bool per_ind, err_se_name=false, se_prov;
        std::vector<std::string> sec_enc;
        joints_ = get_node()->get_parameter("joints").as_string_array();
        if(joints_.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'joints' parameter is empty");
            return CallbackReturn::ERROR;
        }
        se_flag_.resize(joints_.size(),false);
        sec_enc = get_node()->get_parameter("second_encoders").as_string_array();

        if(!sec_enc.empty())
        {
            for(auto &se_name: sec_enc)
            {
                err_se_name = true;
                for(size_t i = 0; i< joints_.size();i++)
                {
                    if(joints_[i] == se_name)
                    {
                        err_se_name = false;
                        se_flag_[i] = true;
                    }
                }
                if(err_se_name)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'second_encoder' named %s are not contained into 'joints' ",se_name.c_str());
                    return CallbackReturn::ERROR;
                }

            }
            se_prov = true;
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger(logger_name_),"'second_encoder' is empty");
            se_prov = false;
        } 
        per_ind = get_node()->get_parameter("performance_index").as_bool();

        stt_msg_.name.resize(joints_.size());
        stt_msg_.position.resize(joints_.size());
        stt_msg_.velocity.resize(joints_.size());
        stt_msg_.effort.resize(joints_.size());
        stt_msg_.temperature.resize(joints_.size());
        stt_msg_.current.resize(joints_.size());
        if(se_prov)
        {
            stt_msg_.sec_enc_pos.resize(joints_.size());
            stt_msg_.sec_enc_vel.resize(joints_.size());
        }

        stt_pub_ = get_node()->create_publisher<StateMsgs>(
            "~/joints_state",
            5
        );

        if(per_ind)
        {
            per_msg_.name.resize(joints_.size());
            per_msg_.pack_loss.resize(joints_.size());
            for(size_t i = 0; i < joints_.size(); i++)
                per_msg_.name[i] = joints_[i];
            
            per_pub_ = get_node()->create_publisher<LossMsgs>(
                "~/performance_indexes",
                5
            );
        }
        RCLCPP_INFO(get_node()->get_logger(),"configurated succesfully");
        return CallbackReturn::SUCCESS;

    }

    CallbackReturn Pi3Hat_State_Broadcaster::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_State_Broadcaster::on_activate(const rclcpp_lifecycle::State &)
    {
        // for(size_t i = 0; i < joints_.size(); i++)
        //     RCLCPP_INFO(get_node()->get_logger(),"joints %d has name %s and is %s",i,joints_[i].c_str(),se_flag_[i]?"active":"inactive");

        RCLCPP_INFO(get_node()->get_logger(),"activated succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_State_Broadcaster::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration Pi3Hat_State_Broadcaster::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration stt_int_cnf;
        stt_int_cnf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::string int_name = "MoteusPi3Hat_Interface";
        stt_int_cnf.names.push_back(int_name + "/" + hardware_interface::HW_IF_VALIDITY_LOSS);
        stt_int_cnf.names.push_back(int_name + "/" + hardware_interface::HW_IF_CYCLE_DUR);
        stt_int_cnf.names.push_back(int_name + "/" + hardware_interface::HW_IF_W2R_DUR);
        for(size_t i = 0; i < joints_.size(); i++)
            stt_int_cnf.names.push_back(joints_[i] + "/" + hardware_interface::HW_IF_PACKAGE_LOSS);
        for(size_t i = 0; i < joints_.size(); i++)
        {
            stt_int_cnf.names.push_back(joints_[i] + "/" + hardware_interface::HW_IF_POSITION);
            stt_int_cnf.names.push_back(joints_[i] + "/" + hardware_interface::HW_IF_VELOCITY);
            stt_int_cnf.names.push_back(joints_[i] + "/" + hardware_interface::HW_IF_EFFORT);
            stt_int_cnf.names.push_back(joints_[i] + "/" + hardware_interface::HW_IF_TEMPERATURE);
            stt_int_cnf.names.push_back(joints_[i] + "/" + hardware_interface::HW_IF_CURRENT);
        }
        for(size_t i = 0; i < joints_.size(); i++)
        {
            if(se_flag_[i])
            {
                stt_int_cnf.names.push_back(joints_[i] + "_ENCODER1/" + hardware_interface::HW_IF_POSITION);
                stt_int_cnf.names.push_back(joints_[i] + "_ENCODER1/" + hardware_interface::HW_IF_VELOCITY);
            }
        }
        // RCLCPP_INFO(get_node()->get_logger(),"the dimesion of hw is %ld",stt_int_cnf.names.size());
        return stt_int_cnf;
    }

    controller_interface::InterfaceConfiguration Pi3Hat_State_Broadcaster::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration cmd_int_cnf;
        cmd_int_cnf.type = controller_interface::interface_configuration_type::NONE;
        return cmd_int_cnf;
    }

    controller_interface::return_type Pi3Hat_State_Broadcaster::update(const rclcpp::Time & time, const rclcpp::Duration &)
    {
        size_t sz = joints_.size();
        if(per_pub_ != nullptr)
        {
            per_msg_.header.set__stamp(time);
            per_msg_.set__valid(state_interfaces_[0].get_value());
            per_msg_.set__cycle_dur(state_interfaces_[1].get_value());
            per_msg_.set__write2read_dur(state_interfaces_[2].get_value());
            // RCLCPP_INFO_STREAM(get_node()->get_logger(),state_interfaces_[2].get_value());
            for(size_t i = 0; i < sz; i++)
            {
                per_msg_.pack_loss[i] = state_interfaces_[i + 3].get_value();
            }

            per_pub_->publish(per_msg_);
        }
        


        stt_msg_.header.set__stamp(time);
        for(size_t i = 0; i < sz; i++)
        {
            // RCLCPP_INFO(get_node()->get_logger(),"executing std stt for jnt %s",joints_[i].c_str());
            stt_msg_.name[i] = joints_[i];
            // RCLCPP_INFO(get_node()->get_logger(),"temp ind is %ld",1 + sz + 4*i );
            stt_msg_.position[i] = state_interfaces_[3 + sz + 5*i ].get_value();
            // RCLCPP_INFO(get_node()->get_logger(),"temp ind is %ld",1 + sz + 4*i + 1);
            stt_msg_.velocity[i] = state_interfaces_[3 + sz + 5*i + 1].get_value();
            // RCLCPP_INFO(get_node()->get_logger(),"temp ind is %ld",1 + sz + 4*i + 2);
            stt_msg_.effort[i] = state_interfaces_[3 + sz + 5*i + 2].get_value();
            // RCLCPP_INFO(get_node()->get_logger(),"temp ind is %ld",1 + sz + 4*i + 3);
            stt_msg_.temperature[i] = state_interfaces_[3 + sz + 5*i + 3].get_value();

            stt_msg_.current[i] = state_interfaces_[3 + sz + 5*i + 4].get_value();
            
        }
    
        for(size_t i = 0; i < sz; i++)
        {
            if(se_flag_[i])
            {
                // RCLCPP_INFO(get_node()->get_logger(),"executing se stt for jnt %s",joints_[i].c_str());
                stt_msg_.sec_enc_pos[i] = state_interfaces_[ 3 + sz*6 + 2*i ].get_value();
                stt_msg_.sec_enc_vel[i] = state_interfaces_[ 3 + sz*6 + 2*i +1].get_value();
            }
        }
        stt_pub_->publish(stt_msg_);

        return controller_interface::return_type::OK;

    }
};

PLUGINLIB_EXPORT_CLASS(
    pi3hat_state_broadcaster::Pi3Hat_State_Broadcaster, controller_interface::ControllerInterface
);