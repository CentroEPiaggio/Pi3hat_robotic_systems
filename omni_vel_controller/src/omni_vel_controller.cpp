#include "omni_vel_controller/omni_vel_controller.hpp"


#include "pluginlib/class_list_macros.hpp"

#include "controller_interface/helpers.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
namespace omni_vel_controller
{
    CallbackReturn Omni_Vel_Controller::on_init()
    {
        try
        {
            auto_declare<int>("input_frequency",100);   
            auto_declare<double>("driveshaft_y",0.188);
            auto_declare<double>("driveshaft_x",0.235);
            auto_declare<double>("mecanum_angle",45.0);
        	auto_declare<double>("wheel_rad",0.05);
            auto_declare<bool>("BestEffort_QOS",true);
            auto_declare<bool>("DeadMiss_event",false);
            auto_declare<bool>("call_dm",false);
            auto_declare<bool>("pub_odom",false);
        }
         catch(const std::exception & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Exception thrown during declaration of joints name with message: %s", e.what());
            return CallbackReturn::ERROR;
        }
        // add physics omni_wheel physics parameter autodeclare
        RCLCPP_INFO(get_node()->get_logger(),"initialize succesfully");
        return CallbackReturn::SUCCESS;
    };
    CallbackReturn Omni_Vel_Controller::on_configure(const rclcpp_lifecycle::State & )
    {
        
        double ds_y,ds_x,ma,wr;
        rclcpp::QoS out_qos(10),in_qos(10);
        // get parameters
        ds_y = get_node()->get_parameter("driveshaft_y").as_double();
        ds_x = get_node()->get_parameter("driveshaft_x").as_double();
        ma = get_node()->get_parameter("mecanum_angle").as_double() *(M_PI/180.0);
        wr = get_node()->get_parameter("wheel_rad").as_double();
        odom_flag_ = get_node()->get_parameter("pub_odom").as_bool();

        milliseconds dur{get_node()->get_parameter("input_frequency").as_int() + 5};
        deadmis_to_ = dur;

        // fill base to wheel kin matrix

        base2Wheel_matrix_[0][0] = 1.0/wr;
        base2Wheel_matrix_[0][1] = 1.0/wr;
        base2Wheel_matrix_[0][2] = -(ds_x - ds_y*(1/std::tan(ma)))/wr;
        

        base2Wheel_matrix_[1][0] = -1.0/wr;
        base2Wheel_matrix_[1][1] = 1.0/wr;
        base2Wheel_matrix_[1][2] = -(ds_x - ds_y*(1/std::tan(ma)))/wr;

        base2Wheel_matrix_[2][0] = -1.0/wr;
        base2Wheel_matrix_[2][1] = -1.0/wr;
        base2Wheel_matrix_[2][2] = -(ds_x - ds_y*(1/std::tan(ma)))/wr;

        base2Wheel_matrix_[3][0] = 1.0/wr;
        base2Wheel_matrix_[3][1] = -1.0/wr;
        base2Wheel_matrix_[3][2] = -(ds_x - ds_y*(1/std::tan(ma)))/wr;

        odom_matrix_[0][0] = (wr /4);
        odom_matrix_[0][1] = -(wr /4);
        odom_matrix_[0][2] = -(wr /4);
        odom_matrix_[0][3] = (wr /4);

        odom_matrix_[1][0] =  (wr /4);
        odom_matrix_[1][1] = (wr /4);
        odom_matrix_[1][2] = -(wr /4);
        odom_matrix_[1][3] = -(wr /4);

        odom_matrix_[2][0] = -(wr /(4*(ds_x - ds_y*(1/std::tan(ma)))));
        odom_matrix_[2][1] = -(wr /(4*(ds_x - ds_y*(1/std::tan(ma)))));
        odom_matrix_[2][2] = -(wr /(4*(ds_x - ds_y*(1/std::tan(ma)))));
        odom_matrix_[2][3] = -(wr /(4*(ds_x - ds_y*(1/std::tan(ma)))));

        joint_cmd_.name.resize(WHEELS);
        joint_cmd_.set__name(wheels_name_);
        joint_cmd_.position.resize(WHEELS);
        joint_cmd_.velocity.resize(WHEELS);
        joint_cmd_.effort.resize(WHEELS);
        joint_cmd_.kp_scale.resize(WHEELS);
        joint_cmd_.kd_scale.resize(WHEELS);

        //set qos protocol
        if(get_node()->get_parameter("BestEffort_QOS").as_bool())
        {
            out_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
            in_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        }
        else
        {
            out_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
            in_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        }
        //set the in qos for deadline miss

        in_qos.deadline(deadmis_to_);
        rclcpp::SubscriptionOptions sub_opt;
        sub_opt.event_callbacks.deadline_callback = 
        [this](rclcpp::QOSDeadlineRequestedInfo & event) 
        {
            this->dl_miss_count_ ++;
            if(dl_miss_count_ > 10)
            {
                std::lock_guard<std::mutex> l_g(var_mutex_);
                c_stt_ = Controller_State::INACTIVE;
            }
            RCLCPP_WARN(this->get_node()->get_logger(),"Passed deadline");
        };
        //create subscriber and publisher
        cmd_sub_ = get_node()->create_subscription<CmdMsg>(
            "~/command",
            in_qos,
            std::bind(
                &Omni_Vel_Controller::cmd_callback,this,std::placeholders::_1),sub_opt);

        joints_cmd_pub_ = get_node()->create_publisher<SttMsg>("~/joints_reference",out_qos);
        if(odom_flag_)
            odom_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>("~/wheel_odom",10);

        // create servicies 

        rclcpp::ServicesQoS srvs_qos;


        homing_serv_ = get_node()->create_service<TransactionService>("~/homing_srv",
        std::bind(&Omni_Vel_Controller::homing_start_srv,this,std::placeholders::_1,std::placeholders::_2),
        srvs_qos.get_rmw_qos_profile()
        );

        emergency_serv_ = get_node()->create_service<TransactionService>("~/emergency_srv",
        std::bind(&Omni_Vel_Controller::emergency_srv,this,std::placeholders::_1,std::placeholders::_2),
        srvs_qos.get_rmw_qos_profile()
        );
        RCLCPP_INFO(get_node()->get_logger(),"configure succesfully");
        return CallbackReturn::SUCCESS;
    }


    CallbackReturn Omni_Vel_Controller::on_activate(const rclcpp_lifecycle::State & )
    {
       
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Omni_Vel_Controller::on_deactivate(const rclcpp_lifecycle::State & )
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Omni_Vel_Controller::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

   controller_interface::InterfaceConfiguration Omni_Vel_Controller::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration stt_int_cnf;
        stt_int_cnf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for( auto & w_name : wheels_name_)
        {
            stt_int_cnf.names.push_back(w_name + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        return stt_int_cnf;
    }

    controller_interface::InterfaceConfiguration Omni_Vel_Controller::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration cmd_int_cnf;
        cmd_int_cnf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(auto &it : wheels_name_)
        {
            cmd_int_cnf.names.push_back(it + "/" + hardware_interface::HW_IF_POSITION);
            cmd_int_cnf.names.push_back(it + "/" + hardware_interface::HW_IF_VELOCITY);
            cmd_int_cnf.names.push_back(it + "/" + hardware_interface::HW_IF_EFFORT);
            cmd_int_cnf.names.push_back(it + "/" + "kp_scale_value");
            cmd_int_cnf.names.push_back(it + "/" + "kd_scale_value");
        }
        return cmd_int_cnf;
    }

    controller_interface::return_type Omni_Vel_Controller::update(
                const rclcpp::Time & time, const rclcpp::Duration & 
            )
    {
        //get wheel velocity and compute the mecanum wheel FK in base frame if requested
        if(odom_flag_)
        {
            for(size_t i = 0; i < wheels_name_.size(); i++)
                wheel_vel_[i] = state_interfaces_[i].get_value();
            omni_fk();
            odom_msg_.header.set__stamp(time);
            odom_msg_.twist.linear.set__x(odom_vel_[0]);
            odom_msg_.twist.linear.set__y(odom_vel_[1]);
            odom_msg_.twist.linear.set__z(0.0);
            odom_msg_.twist.angular.set__x(0.0);
            odom_msg_.twist.angular.set__y(0.0);
            odom_msg_.twist.angular.set__z(odom_vel_[2]);
            odom_pub_->publish(odom_msg_);
            
        }
        // comute the joint wheel velocity reference
        std::lock_guard<std::mutex> lg(var_mutex_);
        switch (c_stt_)
        {
        case Controller_State::INACTIVE:
            
            for(int i = 0;i < WHEELS; i++)
            {
                joint_cmd_.position[i] = 0.0;
                joint_cmd_.velocity[i] = 0.0;
                joint_cmd_.effort[i] = 0.0;
                joint_cmd_.kp_scale[i] = 0.0;
                joint_cmd_.kd_scale[i] = 0.0;
            }

            break;
        case Controller_State::ACTIVE:

            set_cmd(base_vel_);
           
            break;
        default:
        return controller_interface::return_type::ERROR;
            break;
        }

        set_cmd2jnt();
        return controller_interface::return_type::OK;
    }
    
	void    Omni_Vel_Controller::omni_fk()
    {
        for( int i = 0; i < 3; i++)
        {
            odom_vel_[i] = 0.0;
            for(int j =0; j < 4; j++)
            {
                odom_vel_[i] += odom_matrix_[i][j] * wheel_vel_[j];
            }
        }
    }

}; // namespace name
PLUGINLIB_EXPORT_CLASS(
    omni_vel_controller::Omni_Vel_Controller, controller_interface::ControllerInterface
);
