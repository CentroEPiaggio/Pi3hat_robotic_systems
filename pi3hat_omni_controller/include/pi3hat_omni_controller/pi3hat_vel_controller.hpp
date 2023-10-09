#ifndef PI3HAT_VEL_CONTROLLER_HPP
#define PI3HAT_VEL_CONTROLLER_HPP
#include <memory>
#include <string>
#include <vector>
#include <map>
#include "eigen3/Eigen/Dense"
#include <eigen3/Eigen/Core>
#include <mutex>
#include <array>

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
#include "std_srvs/srv/set_bool.hpp"
#include <mutex>

#define LEG_NUM         4
#define JNT_LEG_NUM     2      // 3 Revolute joints each leg 
#define WHL_NUM         4      // 1 wheel for each leg
#define LINK_LENGHT     0.18   // link lenght in m
#define RF_HFE_HOM - 0.785398163
#define RF_KFE_HOM 1.570796326

namespace pi3hat_vel_controller
{
    using VectorXd = Eigen::VectorXd;
    using MatrixXd = Eigen::MatrixXd;
    using CmdMsgs = pi3hat_moteus_int_msgs::msg::OmniMulinexCommand;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using TransactionService = std_srvs::srv::SetBool;
    using namespace std;


    enum Controller_State {PRE_HOMING,HOMING,ACTIVE,EMERGENCY};
    enum LEG_IND {RF,LF,LH,RH};

    class Pi3Hat_Vel_Controller : public controller_interface::ControllerInterface
    {
        public:
            Pi3Hat_Vel_Controller();

            ~Pi3Hat_Vel_Controller()
            {};
            
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

            bool compute_reference(double v_x_tmp, double v_y_tmp, double omega_tmp, double height_rate_tmp, double dt);

            void compute_mecanum_speed(VectorXd& v_base, VectorXd& w_mecanum);


            void compute_leg_joints_vel_ref(VectorXd& q_leg, VectorXd& q_dot_leg, LEG_IND l_index, double height_rate_tmp);

            void homing_start_srv(const shared_ptr<TransactionService::Request> req, 
                                  const shared_ptr<TransactionService::Response> res);
            
            void emergency_srv(const shared_ptr<TransactionService::Request> req, 

                                  const shared_ptr<TransactionService::Response> res);

            // function to compute the next homing reference 
            void compute_homing_ref(LEG_IND l_i);

            // <<-------------------------->>
            // callback and service must be add, i will do it tomorrow

            //<<-------------------------->>
    

            

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
            
            
            std::shared_ptr<CmdMsgs> vel_target_rcvd_msg_;
            bool default_init_pos_;

            // add homing starting times
            std::shared_ptr<rclcpp::Time> homing_start_ = nullptr;

            //robot parameter referring to https://ieeexplore.ieee.org/document/7827337
            double a_, b_, alpha_; 
            // add mutex instance

            std::mutex mutex_var_;
            

            // controller state and and spline parameter declaration
            Controller_State state_ = Controller_State::PRE_HOMING;  
            // given the third order spline p(t) = a_3*t^3 + a_2*t^2 + a_1*t +a_0
            // the parameter contains a_3 and a_2 for RF hip and knee, the others are zero 
            std::array<double,4> spline_par_ = {0.0,0.0,0.0,0.0};
            double homing_dur_ = 0.0;

            std::mutex calbck_m_;
            int loss_counter_ = 0;
            const std::array<LEG_IND,4> legs_ = {RF,LF,LH,RH};
            // joint names has been added, check they are right // just an F instead of an H, for the rest all perfect
            const std::vector<std::string> joints_ = {
                                                    "RF_HFE","RF_HKE",
                                                    "LF_HFE","LF_HKE",
                                                    "LH_HFE","LH_HKE",
                                                    "RH_HFE","RH_HKE",
                                                    "RF_WHEEL","LF_WHEEL",
                                                    "LH_WHEEL","LH_WHEEL"}; 

    };
};


#endif