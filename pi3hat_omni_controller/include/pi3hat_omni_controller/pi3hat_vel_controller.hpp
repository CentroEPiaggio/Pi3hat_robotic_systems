#ifndef PI3HAT_VEL_CONTROLLER_HPP
#define PI3HAT_VEL_CONTROLLER_HPP
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <eigen3/Eigen/Dense>

#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "pi3hat_moteus_int_msgs/msg/joints_command.hpp"
#include "pi3hat_moteus_int_msgs/msg/omni_mulinex_command.hpp" 
#include "realtime_tools/realtime_buffer.h"

#define LEG_NUM         4
#define JNT_LEG_NUM     3      // 3 Revolute joints each leg 
#define WHL_NUM         4      // 1 wheel for each leg
#define LINK_LENGHT     0.18   // link lenght in m

namespace pi3hat_vel_controller
{
    using CmdMsgs = pi3hat_moteus_int_msgs::msg::OmniMulinexCommand;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using namespace std;

    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    class Pi3Hat_Vel_Controller : public controller_interface::ControllerInterface
    {
        public:
            Pi3Hat_Vel_Controller();

            ~Pi3Hat_Vel_Controller();
            
            CallbackReturn on_init() override;
            
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override; 

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(
                const rclcpp::Time & time, const rclcpp::Duration & period
            ) override;

            bool get_target(double& v_x_tmp, double& v_y_tmp, double& omega_tmp, double& height_rate_tmp);

            bool Pi3Hat_Vel_Controller::compute_reference(double v_x_tmp, double v_y_tmp, double omega_tmp, double height_rate_tmp);

            void compute_mecanum_speed(VectorXd& v_base, VectorXd& w_mecanum);

            void compute_leg_joints_vel_ref(VectorXd& q_leg, VectorXd& q_dot_leg, size_t l_index, double height_rate_tmp)

            // le farà jacopino :
            // callback per il servizio di "homing"
            // callback per l'emergenza-> kp_scale e kd_scale a zero 
            // le callback gestiscono anche un membro della classe enum robot_state = {PREHOMING, ACTIVE , EMEEGENCY }
            // PREHOMING non legge i comandi ma aspetta la richiesta di srvizio 
            // ACTIVE risponde alle richieste da topic 
            // EMRGENCY death state 

        private:
            rclcpp::Subscription<CmdMsgs>::SharedPtr cmd_sub_;
            std::string logger_name_;
            std::map<std::string,double> position_cmd_, velocity_cmd_,effort_cmd_,kp_scale_cmd_,kd_scale_cmd_;
            std::map<std::string,double> position_out_, temperature_out_;
            
            realtime_tools::RealtimeBuffer<CmdMsgs>> rt_buffer_;
            
            std::shared_ptr<CmdMsgs> vel_target_rcvd_msg_;
            std::__shared_ptr<>
            bool default_init_pos_;

            //robot parameter referring to https://ieeexplore.ieee.org/document/7827337
            double a_, b_, alpha_; 
            // add mutex instance
            const std::vector<std::string> joints_ = {"jnt1","jnt2",..}   
    };
};


#endif