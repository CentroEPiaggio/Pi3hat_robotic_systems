#include "gtest/gtest.h"
#include "pi3hat_hw_interface/motor_manager.hpp"
#include <functional>
#include<vector>
#include <array>
#include <random>
using motor_mem = pi3hat_hw_interface::motor_manager::motor_mem;
using Reply = moteus::Pi3HatMoteusInterface::ServoReply;
using Get_Function = std::function<moteus::QueryResultV2 ( std::vector<Reply>& replies, int bus, int id, int opt,int& err)>;
using Policy_Function = std::function<void(bool msg_valid, bool msg_coplete, Command* cmd_d)>;

TEST(motor_manager_test, first_test)
{
  EXPECT_EQ(4, 2 + 2);
}
TEST(motor_manager_test, correct_cmd_res)
{
    moteus::PositionResolution T_1;
    moteus::QueryCommandV2 T_2;

    uniform_real_distribution<double> unif(-5.0,5.0);
    default_random_engine re;

    uniform_int_distribution<uint16_t> unif2(0,10);
    uint8_t id = 4;
    uint8_t bus = 1;
    std::array<double,4> msr_var = {0.0,0.0,0.0,0.0};
    std::array<double,5> cmd_var = {unif(re),unif(re),unif(re),unif(re),unif(re)};
    std::array<uint16_t,2> scale = {unif2(re),unif2(re)};
    moteus::Pi3HatMoteusInterface::ServoCommand cmd_data;
    std::vector<moteus::Pi3HatMoteusInterface::ServoReply> msr_data;
    msr_data.resize(3);
    pi3hat_hw_interface::motor_manager::motor_info a;
    motor_mem mem;
    mem.cmd_pos = &cmd_var[0];
    mem.cmd_vel = &cmd_var[1];
    mem.cmd_trq = &cmd_var[2];
    mem.kp_scale = &cmd_var[3];
    mem.kd_scale = &cmd_var[4];
    mem.cmd_data = &cmd_data;
    mem.msr_pos = &msr_var[0];
    mem.msr_vel = &msr_var[1];
    mem.msr_trq = &msr_var[2];
    mem.msr_tmp = &msr_var[3];
    mem.replies = &msr_data;
    Get_Function gets = [] (std::vector<Reply>& replies,int bus,int id,int opt,int& err) -> moteus::QueryResultV2
    {
        
      for (const auto& item : replies) 
                  {
                  if (item.id == id && item.bus == bus)
                  { 
                    err = 0;
                    return item.result; }
                  }
                  err = 1;
                  return {};
    } ;
    Policy_Function pol = [](bool msg_valid, bool msg_complete,Command* cmd_d)
    {


    };



    pi3hat_hw_interface::motor_manager::Motor_Manager istance(mem,false,id,bus,pol,gets);

    T_1.position = moteus::Resolution::kFloat;
    T_1.velocity = moteus::Resolution::kFloat;
    T_1.feedforward_torque = moteus::Resolution::kFloat;
    T_1.kp_scale = moteus::Resolution::kInt16;
    T_1.kd_scale = moteus::Resolution::kInt16;

    istance.set_command_resolution(T_1);

    moteus::PositionResolution R_1= istance.get_cmd_res();

    EXPECT_EQ(R_1.position,T_1.position);
    EXPECT_EQ(R_1.velocity,T_1.velocity);
    EXPECT_EQ(R_1.feedforward_torque,T_1.feedforward_torque);
    EXPECT_EQ(R_1.kd_scale,T_1.kd_scale);
    EXPECT_EQ(R_1.kp_scale,T_1.kp_scale);
    EXPECT_EQ(R_1.maximum_torque,T_1.maximum_torque);
    EXPECT_EQ(R_1.stop_position,T_1.stop_position);
    EXPECT_EQ(R_1.watchdog_timeout,T_1.watchdog_timeout);


    T_2.position = moteus::Resolution::kFloat;
    T_2.velocity = moteus::Resolution::kFloat;
    T_2.torque = moteus::Resolution::kFloat;
    T_2.temperature = moteus::Resolution::kInt16;
    istance.set_query_resolution(T_2);
    moteus::QueryCommandV2 R_2 = istance.get_qry_res();
  //  std::cerr << "complete set_command_resolution_test"<<std::endl;
    EXPECT_EQ(T_2.position,R_2.position);
    EXPECT_EQ(T_2.velocity,R_2.velocity);
    EXPECT_EQ(T_2.d_current,R_2.d_current);
    EXPECT_EQ(T_2.mode,R_2.mode);
    EXPECT_EQ(T_2.q_current,R_2.q_current);
    EXPECT_EQ(T_2.rezero_state,R_2.rezero_state);
    EXPECT_EQ(T_2.temperature,R_2.temperature);
    EXPECT_EQ(T_2.torque,R_2.torque);
    EXPECT_EQ(T_2.velocity,R_2.velocity);
    EXPECT_EQ(T_2.voltage,R_2.voltage);

    // test the make stop method

    istance.make_stop();
    EXPECT_EQ(cmd_data.id,istance.get_id());
    EXPECT_EQ(cmd_data.bus,istance.get_bus());
    EXPECT_EQ(cmd_data.mode,moteus::Mode::kStopped);

    // EXPECT_TRUE(false) << "prova msg\n";
    //test the make positions

    istance.make_position();
    EXPECT_EQ(cmd_data.id,istance.get_id());
    EXPECT_EQ(cmd_data.bus,istance.get_bus());
    EXPECT_EQ(cmd_data.mode,moteus::Mode::kPosition);
    EXPECT_EQ(cmd_data.position.position,cmd_var[0]);
    EXPECT_EQ(cmd_data.position.velocity,cmd_var[1]);
    EXPECT_EQ(cmd_data.position.feedforward_torque,cmd_var[2]);
    EXPECT_EQ(cmd_data.position.kp_scale,cmd_var[3]);
    EXPECT_EQ(cmd_data.position.kd_scale,cmd_var[4]);
    // EXPECT_TRUE(false) << cmd_data.position.kp_scale<<" :: "<<cmd_var[3]<<endl<<
    // cmd_data.position.kd_scale<<" :: "<<cmd_var[4]<<endl;    // 
    

    EXPECT_EQ(R_1.position,cmd_data.resolution.position);
    EXPECT_EQ(R_1.velocity,cmd_data.resolution.velocity);
    EXPECT_EQ(R_1.feedforward_torque,cmd_data.resolution.feedforward_torque);
    EXPECT_EQ(R_1.kd_scale,cmd_data.resolution.kd_scale);
    EXPECT_EQ(R_1.kp_scale,cmd_data.resolution.kp_scale);
    EXPECT_EQ(R_1.maximum_torque,cmd_data.resolution.maximum_torque);
    EXPECT_EQ(R_1.stop_position,cmd_data.resolution.stop_position);
    EXPECT_EQ(R_1.watchdog_timeout,cmd_data.resolution.watchdog_timeout);


    EXPECT_EQ(cmd_data.query.position,R_2.position);
    EXPECT_EQ(cmd_data.query.velocity,R_2.velocity);
    EXPECT_EQ(cmd_data.query.d_current,R_2.d_current);
    EXPECT_EQ(cmd_data.query.mode,R_2.mode);
    EXPECT_EQ(cmd_data.query.q_current,R_2.q_current);
    EXPECT_EQ(cmd_data.query.rezero_state,R_2.rezero_state);
    EXPECT_EQ(cmd_data.query.temperature,R_2.temperature);
    EXPECT_EQ(cmd_data.query.torque,R_2.torque);
    EXPECT_EQ(cmd_data.query.velocity,R_2.velocity);
    EXPECT_EQ(cmd_data.query.voltage,R_2.voltage);


    // test get_motor_state()
    std::array<uint8_t,3> ids = {id+1,id-1,id};
    std::array<uint8_t,3> buss = {bus,bus,bus};
    std::array<double,3> poss = {unif(re),unif(re),unif(re)};
    std::array<double,3> vels = {unif(re),unif(re),unif(re)};
    std::array<double,3> trqs = {unif(re),unif(re),unif(re)};
    std::array<double,3> tmps = {unif(re),unif(re),unif(re)};
    std::array<uint8_t,3> faults = {0,0,0};

    for(int i = 0;i < 3; i++)
    {
      msr_data[i].id = ids[i];
      msr_data[i].bus = buss[i];
      msr_data[i].result.position = poss[i];
      msr_data[i].result.velocity = vels[i];
      msr_data[i].result.torque = trqs[i];
      msr_data[i].result.temperature = tmps[i];
      msr_data[i].result.fault = faults[i];
    }
    int res = istance.get_motor_state();

    EXPECT_EQ(res,faults[2]);
    EXPECT_EQ(ids[2],istance.get_id());
    EXPECT_EQ(buss[2],istance.get_bus());
    EXPECT_EQ(poss[2],msr_var[0]);
    EXPECT_EQ(vels[2],msr_var[1]);
    EXPECT_EQ(trqs[2],msr_var[2]);
    EXPECT_EQ(tmps[2],msr_var[3]);
}
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
