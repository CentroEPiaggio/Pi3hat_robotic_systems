#include "gtest/gtest.h"
#include "pi3hat_hw_interface/motor_manager.hpp"
#include <functional>
#include<vector>
#include <array>
using motor_mem = pi3hat_hw_interface::motor_manager::motor_mem;
using Reply = moteus::Pi3HatMoteusInterface::ServoReply;
using Get_Function = std::function<moteus::QueryResultV2 ( std::vector<Reply>& replies, int bus, int id, int opt,int& err)>;
using Policy_Function = std::function<void(bool msg_valid, bool msg_coplete, Command* cmd_d)>;

TEST(motor_manager_test, first_test)
{
  ASSERT_EQ(4, 2 + 2);
}
// TEST(motor_manager_test, correct_cmd_res)
// {
//     moteus::PositionResolution T_1;
//     std::array<double,4> msr_var = {0.0,0.0,0.0,0.0};
//     std::array<double,5> cmd_var = {0.0,0.0,0.0,0.0,0.0};
//     moteus::Pi3HatMoteusInterface::ServoCommand cmd_data;
//     std::vector<moteus::Pi3HatMoteusInterface::ServoReply> msr_data;
//     msr_data.resize(1);
//     pi3hat_hw_interface::motor_manager::motor_info a;
//     motor_mem mem;
//     mem.cmd_pos = &cmd_var[0];
//     mem.cmd_vel = &cmd_var[1];
//     mem.cmd_trq = &cmd_var[2];
//     mem.kp_scale = &cmd_var[3];
//     mem.kd_scale = & cmd_var[4];
//     mem.cmd_data = &cmd_data;
//     mem.msr_pos = &msr_var[0];
//     mem.msr_vel = &msr_var[1];
//     mem.msr_trq = &msr_var[2];
//     mem.msr_tmp = &msr_var[3];
//     mem.replies = &msr_data;
//     Get_Function gets = [] (std::vector<Reply>& replies,int bus,int id,int opt,int& err) -> moteus::QueryResultV2
//     {
        
//         return {};
//     } ;
//     Policy_Function pol = [](bool msg_valid, bool msg_complete,Command* cmd_d)
//     {

//     };



//     pi3hat_hw_interface::motor_manager::Motor_Manager istance(mem,false,1,1,pol,gets);

//     T_1.position = moteus::Resolution::kFloat;
//     T_1.velocity = moteus::Resolution::kFloat;
//     T_1.feedforward_torque = moteus::Resolution::kFloat;
//     T_1.kp_scale = moteus::Resolution::kInt16;
//     T_1.kd_scale = moteus::Resolution::kInt16;

//     istance.set_command_resolution(T_1);

//     moteus::PositionResolution R_1= istance.get_cmd_res();

    // ASSERT_EQ(R_1.position,T_1.position);
    // ASSERT_EQ(R_1.velocity,T_1.velocity);
    // ASSERT_EQ(R_1.feedforward_torque,T_1.feedforward_torque);
    // ASSERT_EQ(R_1.kd_scale,T_1.kd_scale);
    // ASSERT_EQ(R_1.kp_scale,T_1.kp_scale);
    // ASSERT_EQ(R_1.maximum_torque,T_1.maximum_torque);
    // ASSERT_EQ(R_1.stop_position,T_1.stop_position);
    // ASSERT_EQ(R_1.watchdog_timeout,T_1.watchdog_timeout);
   //std::cout << "complete set_command_resolution_test"<<std::endl;

// }
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
