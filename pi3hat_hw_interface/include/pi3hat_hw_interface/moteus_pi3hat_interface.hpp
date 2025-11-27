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
#include "tf2/tf2/LinearMath/Quaternion.h"
#include "tf2_ros/tf2_ros/transform_broadcaster.h"
#define DEBUG_IMU true

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "rclcpp_lifecycle/lifecycle_publisher.hpp"
// #include "pi3hat_hw_interface/motor_manager.hpp"
// #include "pi3hat_moteus_int_msgs/msg/packet_pass.hpp"
// #include "moteus_pi3hat/pi3hat_moteus_interface.h"
#define NUM_STOP 30
#define MAIN_TIMEOUT 3000000
#define MIN_TX_TIMEOUT 200000
#define PI_ 3.14159265358979323846

using namespace mjbots;
using pi3hat_hw_interface::motor_manager::Motor_Manager;
using MoteusInterface = moteus::Pi3HatMoteusInterface;
using Command = moteus::Pi3HatMoteusInterface::ServoCommand;
using Reply = moteus::Pi3HatMoteusInterface::ServoReply;
using Options = moteus::Pi3HatMoteusInterface::Options;
using Data = moteus::Pi3HatMoteusInterface::Data;
using Output = moteus::Pi3HatMoteusInterface::Output;
using PerData = pi3hat_moteus_int_msgs::msg::PacketPass;
using namespace std::chrono_literals;

#define SLEEP_FOR_10MS 10000000ns
#define NUM_ACC 3
#define NUM_VEL 3

using Get_Function = std::function<moteus::QueryResultV2 ( std::vector<Reply>& replies, int bus, int id, int opt,int& err,int provided_msg)>;
using Policy_Function = std::function<void( bool msg_valid, bool msg_coplete, Command* cmd_d)>;

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

                // void cycle()
                // {
                //     if(valid_)
                //     {
                //         auto promise = make_shared<std::promise<Output>>();
                //         for(auto &rep : msr_data_)
                //         {
                //             rep.id = 0;
                //             rep.bus = 0;
                //             rep.result.position = std::nan("1");
                //             rep.result.velocity = std::nan("2");
                //             rep.result.torque = std::nan("3");
                //             rep.result.temperature = std::nan("4");
                //             rep.result.sec_enc_pos = std::nan("6");
                //             rep.result.sec_enc_vel = std::nan("2");
                //         }
                //         data_.commands = {cmd_data_.data(),cmd_data_.size()};
                //         //  for(auto cmd : data_.commands)
                //         //     {
                //         //     RCLCPP_INFO(rclcpp::get_logger("LIV10"),"value of data is %d",cmd.query.sec_enc_pos);
                //         //     }
                //         data_.replies  = {msr_data_.data(),msr_data_.size()};
                //         communication_thread_.Cycle(
                //                 data_,
                //                 [promise](const Output& out)
                //                 { 
                //                     promise->set_value(out);
                //                     // if(out.query_result_size <4 || out.query_result_size >4)
                //                     // 
                //                     // RCLCPP_WARN(rclcpp::get_logger("PINO"),"CALL Communication Callback with out %ld",out.query_result_size);
                //                 }
                //             );
                            
                //         can_recvd_ = promise->get_future();
                //         //RCLCPP_INFO(rclcpp::get_logger("LOGGER_NAME"),"Cycle Call valid %d", can_recvd_.valid());
                //         // RCLCPP_INFO(rclcpp::get_logger("LOGGER_NAME"),"Cycle Call gets %d", can_recvd_.get().query_result_size);
                //     }
                // };
            private:
                
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
                // mjbots::pi3hat::Attitude filtered_IMU_;
                // int num_stt_int_;
                // std::vector<double> acc_base_,vel_base_,quaternion_;
                // Eigen::Vector3d acc_imu_,vel_imu_,imu_to_base_pos_;
                // Eigen::Quaternion<double> orientation_, imuw2_nav_;
                // int acc_correction_ = 0;
                // tf2::Quaternion imu_pose_;
                // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
                // std::chrono::_V2::system_clock::time_point t_e_write_ , t_s_read_;
                // double w2r_dur_;
                // bool first_cycle_ = true;
        };
    }
}



#endif