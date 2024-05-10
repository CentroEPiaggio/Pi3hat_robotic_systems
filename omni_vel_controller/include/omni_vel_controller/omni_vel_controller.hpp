#ifndef OMNI_VEL_ROS2_HPP
#define OMNI_VEL_ROS2_HPP
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <Eigen/Core>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "pi3hat_moteus_int_msgs/msg/omni_mulinex_command.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <chrono>
#include <mutex>


#define WHEELS 4
#define BASE_REF 3



using namespace std::chrono;



namespace omni_vel_controller
{

    enum Controller_State{
        INACTIVE = 0,
        ACTIVE,
    };

    using CmdMsg = pi3hat_moteus_int_msgs::msg::OmniMulinexCommand;
    using SttMsg = pi3hat_moteus_int_msgs::msg::JointsCommand;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using TransactionService = std_srvs::srv::SetBool;

    class Omni_Vel_Controller : public controller_interface::ControllerInterface
    {   
        public:
            Omni_Vel_Controller(){};
            ~Omni_Vel_Controller(){};

            
            CallbackReturn on_init() override;
            
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;


            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override; 

            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override; 

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(
                const rclcpp::Time & time, const rclcpp::Duration & period
            ) override;

        private:

            void cmd_callback(const std::shared_ptr<CmdMsg> msg)
            {
                std::lock_guard<std::mutex> lg(var_mutex_);
                dl_miss_count_ = 0;
                if(c_stt_ == Controller_State::ACTIVE)
                {
                    base_vel_[0] = msg->v_x;
                    base_vel_[1] = msg->v_y;
                    base_vel_[2] = msg->omega;
                }
            }
            void set_cmd2jnt()
            {
                
                for(size_t i = 0 ; i < wheels_name_.size(); i++)
                {
                    command_interfaces_[5 * i].set_value(joint_cmd_.position[i]);
                    command_interfaces_[5 * i + 1].set_value(joint_cmd_.velocity[i]);
                    command_interfaces_[5 * i + 2].set_value(0.0);
                    command_interfaces_[5 * i + 3].set_value(0.0);
                    command_interfaces_[5 * i + 4].set_value(1.0);
                }
            }
            void set_cmd(double base_vel[3])
            {
                double w_v;
                for(int i = 0; i < WHEELS; i++)
                {
                    w_v = 0.0;
                    for(int j = 0; j < BASE_REF; j++)
                    {
                      w_v += base2Wheel_matrix_[i][j]*base_vel[j];
                    }
                    joint_cmd_.position[i] = std::nan("1");
                    joint_cmd_.velocity[i] = w_v;
                    joint_cmd_.effort[i] = 0.0;
                    joint_cmd_.kp_scale[i] = 1.0;
                    joint_cmd_.kd_scale[i] = 1.0;
                }
            }

            void emergency_srv(const std::shared_ptr<TransactionService::Request> req, 
                                  const std::shared_ptr<TransactionService::Response> res)
            {
                std::lock_guard<std::mutex> lg(var_mutex_);
                // RCLCPP_INFO_STREAM(get_node()->get_logger(), req->data);
                // RCLCPP_INFO_STREAM(get_node()->get_logger(), c_stt_)
                if(c_stt_ != Controller_State::INACTIVE && req->data)
                {
                    c_stt_ = Controller_State::INACTIVE;
                    res ->success = true;
                    res ->message = std::string("Emergency mode has been activated");

                }
                else
                {
                    res ->success = false;
                    if(req->data)
                        res ->message = std::string("The state is just in Emergency mode");
                    else
                        res -> message = std::string("Request is not correct");                
                }
            }

            void  homing_start_srv(const std::shared_ptr<TransactionService::Request> req, 
                                  const std::shared_ptr<TransactionService::Response> res)
            {
                std::lock_guard<std::mutex> lg(var_mutex_);
                if(c_stt_ != Controller_State::ACTIVE && req->data)
                {
                    c_stt_ = Controller_State::ACTIVE;
                    res ->success = true;
                    res ->message = std::string("Active mode has been activated");
                }
                else
                {
                    res ->success = true;
                    if(req->data)
                        res ->message = std::string("The state is just in Active mode");
                    else
                        res -> message = std::string("Request is not correct");  
                }

            }
            rclcpp::Publisher<SttMsg>::SharedPtr joints_cmd_pub_;
            rclcpp::Subscription<CmdMsg>::SharedPtr cmd_sub_;
            double base2Wheel_matrix_[4][3];
            double base_vel_[3] = {0.0,0.0,0.0};
            SttMsg joint_cmd_;
            std::string logger_name_;
            std::mutex var_mutex_;
            duration<double,std::milli> deadmis_to_;
            Controller_State c_stt_ = Controller_State::INACTIVE;
            int dl_miss_count_ = 0;
            std::vector<std::string> wheels_name_ = {"RF_WHEEL","LF_WHEEL","LH_WHEEL","RH_WHEEL"};
           
            rclcpp::Service<TransactionService>::SharedPtr homing_serv_,emergency_serv_;
    };
};


#endif
