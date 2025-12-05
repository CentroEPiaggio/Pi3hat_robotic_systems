#ifndef MOTEUS_PI3HAT_HW_INT_HPP
#define MOTEUS_PI3HAT_HW_INT_HPP
#include "hardware_interface/system.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "pi3hat_hw_interface/motor_manager.hpp"
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <chrono>
#include <future>
#include <functional>
#include <stdio.h>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"
#include "hardware_interface/handle.hpp"

#define DEBUG_IMU true

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "moteus_pi3hat/pi3hat_moteus_transport.h"
#include "pi3hat_hw_interface/actuator_manager.hpp"
// #include "rclcpp_lifecycle/lifecycle_publisher.hpp"
// #include "pi3hat_hw_interface/motor_manager.hpp"
// #include "pi3hat_moteus_int_msgs/msg/packet_pass.hpp"
// #include "moteus_pi3hat/pi3hat_moteus_interface.h"
#define NUM_STOP 30
#define MAIN_TIMEOUT 3000000
#define MIN_TX_TIMEOUT 200000
#define PI_ 3.14159265358979323846

// using namespace mjbots;
// using pi3hat_hw_interface::motor_manager::Motor_Manager;
// using MoteusInterface = moteus::Pi3HatMoteusInterface;
// using Command = moteus::Pi3HatMoteusInterface::ServoCommand;
// using Reply = moteus::Pi3HatMoteusInterface::ServoReply;
// using Options = moteus::Pi3HatMoteusInterface::Options;
// using Data = moteus::Pi3HatMoteusInterface::Data;
// using Output = moteus::Pi3HatMoteusInterface::Output;
// using PerData = pi3hat_moteus_int_msgs::msg::PacketPass;
using namespace std::chrono_literals;

#define SLEEP_FOR_10MS 10000000ns
#define NUM_ACC 3
#define NUM_VEL 3

// using Get_Function = std::function<moteus::QueryResultV2 ( std::vector<Reply>& replies, int bus, int id, int opt,int& err,int provided_msg)>;
// using Policy_Function = std::function<void( bool msg_valid, bool msg_coplete, Command* cmd_d)>;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace hardware_interface
{
    constexpr char HW_IF_LIN_ACC_X[] = "linear_acceleration.x";
    constexpr char HW_IF_LIN_ACC_Y[] = "linear_acceleration.y";
    constexpr char HW_IF_LIN_ACC_Z[] = "linear_acceleration.z";

    constexpr char HW_IF_ANG_SPD_X[] = "angular_velocity.x";
    constexpr char HW_IF_ANG_SPD_Y[] = "angular_velocity.y";
    constexpr char HW_IF_ANG_SPD_Z[] = "angular_velocity.z";

    constexpr char HW_IF_QUATERN_X[] = "orientation.x";
    constexpr char HW_IF_QUATERN_Y[] = "orientation.y";
    constexpr char HW_IF_QUATERN_Z[] = "orientation.z";
    constexpr char HW_IF_QUATERN_W[] = "orientation.w";

    

}

namespace pi3hat_hw_interface
{
    namespace moteus_pi3hat_interface
    {
        class AsyncCallback
        {
            public:
                mjbots::moteus::CompletionCallback callback()
                {
                    return [&](int v)
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        // RCLCPP_INFO(rclcpp::get_logger("CALLBACK"),"completition callback");
                        done_.store(true);
                        result_.store(v,std::memory_order_release);
                    };
                };
                int try_consume()
                {
                    //preliminary check to not direct block the mutex
                    if(!done_.load(std::memory_order_acquire))
                    {
                        // RCLCPP_INFO(rclcpp::get_logger("CALLBACK"),"no data yet");
                        return -1;
                    }
                    // lock the mutex in the actual scope
                    std::lock_guard<std::mutex> lock(mutex_);
                    done_.store(false);
                    // RCLCPP_INFO(rclcpp::get_logger("CALLBACK"),"try consume %d", result_.load(std::memory_order_acquire));

                    return result_.load(std::memory_order_acquire);
                    
                }
            private:
                std::atomic<bool> done_{false};
                std::atomic<int> result_{0};
                std::mutex mutex_;
        };
        
        class MoteusPi3Hat_Interface : public hardware_interface::SystemInterface
        {
            
            public:
                MoteusPi3Hat_Interface();
                ~MoteusPi3Hat_Interface();
                CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
                CallbackReturn on_configure(const rclcpp_lifecycle::State& ) override;
                CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_error(const rclcpp_lifecycle::State&) override;

                std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
                std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

                hardware_interface::return_type read(const rclcpp::Time & , const rclcpp::Duration & ) override;
                hardware_interface::return_type write(const rclcpp::Time & , const rclcpp::Duration & ) override;

               
            private:
                std::vector<std::unique_ptr<pi3hat_hw_interface::actuator_manager::Actuator_Manager>> actuators_;
                std::shared_ptr<mjbots::pi3hat::Pi3HatMoteusTransport> pi3hat_transport_;
                unsigned int num_actuators_ = 0;
                std::vector<mjbots::moteus::CanFdFrame> command_framees_,replies_;
                // std::vector<Motor_Manager> motors_;
                // std::vector<Command> cmd_data_;
                // std::vector<Reply> msr_data_;
                // Options opt_;
                // MoteusInterface communication_thread_;
                // Get_Function gets_;
                // Policy_Function poly_;
                // Data data_;
                // Output out_;
                // Options opt_thread_;
                // std::future<Output> can_recvd_;
                // int count_ = 0 ;
                // int not_val_cycle_ = 0,epoch_count_=0;
                // std::vector<double> pkt_loss_;
                // double valid_loss_ = 0.0, cycle_dur_=0.0;
                // bool valid_ = true,att_req_;
                mjbots::pi3hat::Attitude filtered_IMU_;
                mjbots::pi3hat::Quaternion imu_orientation_;
                mjbots::pi3hat::Point3D imu_angular_velocity_, imu_linear_acceleration_;
                // int num_stt_int_;
                // std::vector<double> acc_base_,vel_base_,quaternion_;
                // Eigen::Vector3d acc_imu_,vel_imu_,imu_to_base_pos_;
                // Eigen::Quaternion<double> orientation_, imuw2_nav_;
                // int acc_correction_ = 0;
                // tf2::Quaternion imu_pose_;
                // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
                // std::chrono::_V2::system_clock::time_point t_e_write_ , t_s_read_;
                // double w2r_dur_;
                double invalid_cycle_;
                double cycle_duration_;
                std::vector<double> package_loss_;
                bool first_cycle_ = true, attittude_requested_ = false;
                AsyncCallback clb_as_;
        };
    }
}



#endif