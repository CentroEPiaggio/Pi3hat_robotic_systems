#include "gtest/gtest.h"
#include "pi3hat_hw_interface/motor_manager.hpp"
#include <functional>
#include<vector>
#include <array>
#include <random>
#include <string>
#include <cstdio>
#define NUM_REP_TEST 6
using motor_mem = pi3hat_hw_interface::motor_manager::motor_mem;
using Reply = moteus::Pi3HatMoteusInterface::ServoReply;
using Get_Function = std::function<moteus::QueryResultV2 ( std::vector<Reply>& replies, int bus, int id, int opt,int& err,int prov_msg)>;
using Policy_Function = std::function<void(bool msg_valid, bool msg_coplete, Command* cmd_d)>;

TEST(motor_manager_test, first_test)
{
  EXPECT_EQ(4, 2 + 2);
}

TEST(motor_manager_test,Presolutio_eq_operator)
{
    moteus::PositionResolution T1,T2;
    EXPECT_TRUE(T1==T2);
    T1.position = moteus::Resolution::kInt16;
    T1.velocity = moteus::Resolution::kInt16;
    T1.feedforward_torque = moteus::Resolution::kInt16;
    T1.kd_scale = moteus::Resolution::kInt16;
    T1.kp_scale = moteus::Resolution::kInt16;
    T1.maximum_torque = moteus::Resolution::kInt16;
    T1.stop_position = moteus::Resolution::kInt16;
    T1.watchdog_timeout = moteus::Resolution::kInt16;

    T2.position = moteus::Resolution::kInt16;
    T2.velocity = moteus::Resolution::kInt16;
    T2.feedforward_torque = moteus::Resolution::kInt16;
    T2.kd_scale = moteus::Resolution::kInt16;
    T2.kp_scale = moteus::Resolution::kInt16;
    T2.maximum_torque = moteus::Resolution::kInt16;
    T2.stop_position = moteus::Resolution::kInt16;
    T2.watchdog_timeout = moteus::Resolution::kInt16;
    EXPECT_TRUE(T1==T2);
}

TEST(motor_manager_test,Qresolutio_eq_operator)
{
    moteus::QueryCommandV2 T1,T2;
    EXPECT_TRUE(T1==T2);
    T1.position = moteus::Resolution::kInt16;
    T1.velocity = moteus::Resolution::kInt16;
    T1.torque = moteus::Resolution::kInt16;
    T1.sec_enc_pos = moteus::Resolution::kInt16;
    T1.sec_enc_vel = moteus::Resolution::kInt16;
    T2.position = moteus::Resolution::kInt16;
    T2.velocity = moteus::Resolution::kInt16;
    T2.torque = moteus::Resolution::kInt16;
    T2.sec_enc_pos = moteus::Resolution::kInt16;
    T2.sec_enc_vel = moteus::Resolution::kInt16;
    EXPECT_TRUE(T1==T2);
}

TEST(motor_manager_test,set_cmd_res)
{
  moteus::PositionResolution T1,T2;
  moteus::Pi3HatMoteusInterface::ServoCommand cmd_data;
  std::vector<moteus::Pi3HatMoteusInterface::ServoReply> msr_data;
    Get_Function gets = [] (std::vector<Reply>& replies,int bus,int id,int opt,int& err, int prov_msg ) -> moteus::QueryResultV2
  {
      int i = 0;
    for (const auto& item : replies) 
    {
      i++;
      if(i>prov_msg)
        break;
      if (item.id == id && item.bus == bus)
      { 
        err = 1;
        return item.result; 
      }
    }
    err = 2;
    return {};
  } ;
  Policy_Function pol = [](bool msg_valid, bool msg_complete,Command* cmd_d)
  {


  };
  uint8_t id = 1;
  uint8_t bus = 1;
  std::string name = "Jnt_test";
  double motor_trans = 1.0, sec_enc_trasn = 0.0;
  pi3hat_hw_interface::motor_manager::Motor_Manager test(
          name,
          &cmd_data,
          &msr_data,
          motor_trans,
          sec_enc_trasn,
          id,
          bus,
          pol,
          gets);
  test.set_command_resolution(T1);
  T2 = test.get_cmd_res();
  EXPECT_TRUE(T1==T2);
  T1.position = moteus::Resolution::kInt16;
  T1.velocity = moteus::Resolution::kInt16;
  T1.feedforward_torque = moteus::Resolution::kInt16;
  T1.kd_scale = moteus::Resolution::kInt16;
  T1.kp_scale = moteus::Resolution::kInt16;
  T1.maximum_torque = moteus::Resolution::kInt16;
  T1.stop_position = moteus::Resolution::kInt16;
  T1.watchdog_timeout = moteus::Resolution::kInt16;
  test.set_command_resolution(T1);
  T2 = test.get_cmd_res();
  EXPECT_TRUE(T1==T2);







  EXPECT_TRUE(true);
}


TEST(motor_manager_test,set_stt_res)
{
  moteus::QueryCommandV2 T1,T2;
  EXPECT_TRUE(T1==T2);
  moteus::Pi3HatMoteusInterface::ServoCommand cmd_data;
  std::vector<moteus::Pi3HatMoteusInterface::ServoReply> msr_data;
  Get_Function gets = [] (std::vector<Reply>& replies,int bus,int id,int opt,int& err, int prov_msg ) -> moteus::QueryResultV2
  {
      int i = 0;
    for (const auto& item : replies) 
    {
      i++;
      if(i>prov_msg)
        break;
      if (item.id == id && item.bus == bus)
      { 
        err = 1;
        return item.result; 
      }
    }
    err = 2;
    return {};
  } ;
  Policy_Function pol = [](bool msg_valid, bool msg_complete,Command* cmd_d)
  {


  };
  uint8_t id = 1;
  uint8_t bus = 1;
  std::string name = "Jnt_test";
  double motor_trans = 1.0, sec_enc_trasn = 0.0;
  pi3hat_hw_interface::motor_manager::Motor_Manager test(
          name,
          &cmd_data,
          &msr_data,
          motor_trans,
          sec_enc_trasn,
          id,
          bus,
          pol,
          gets);
  test.set_query_resolution(T1);
  T2 = test.get_qry_res();
  EXPECT_TRUE(T1==T2);
  T1.position = moteus::Resolution::kInt16;
  T1.velocity = moteus::Resolution::kInt16;
  T1.torque = moteus::Resolution::kInt16;
  T1.voltage = moteus::Resolution::kInt16;
  T1.temperature = moteus::Resolution::kInt16;
  test.set_query_resolution(T1);
  T2 = test.get_qry_res();
  EXPECT_TRUE(T1==T2);
  EXPECT_TRUE(true);
}

TEST(motor_manager_test,exp_cmd_int)
{
  uniform_real_distribution<double> unif(-5.0,5.0);
  default_random_engine re;
  moteus::Pi3HatMoteusInterface::ServoCommand cmd_data;
  std::vector<moteus::Pi3HatMoteusInterface::ServoReply> msr_data;
  Get_Function gets = [] (std::vector<Reply>& replies,int bus,int id,int opt,int& err, int prov_msg ) -> moteus::QueryResultV2
  {
      int i = 0;
    for (const auto& item : replies) 
    {
      i++;
      if(i>prov_msg)
        break;
      if (item.id == id && item.bus == bus)
      { 
        err = 1;
        return item.result; 
      }
    }
    err = 2;
    return {};
  } ;
  Policy_Function pol = [](bool msg_valid, bool msg_complete,Command* cmd_d)
  {


  };
  uint8_t id = 1;
  uint8_t bus = 1;
  std::string name = "Jnt_test";
  double motor_trans = 1.0, sec_enc_trasn = 0.0;
  pi3hat_hw_interface::motor_manager::Motor_Manager test(
          name,
          &cmd_data,
          &msr_data,
          motor_trans,
          sec_enc_trasn,
          id,
          bus,
          pol,
          gets);
  
  std::vector<std::string> int_type = test.get_command_type();
  interface_tpl res;
  std::printf("PASSO\n");
  int i = 0;
  for(auto inter : int_type)
  {
      EXPECT_TRUE(name == test.get_name(false));
      std::printf("interface %s\n",inter.c_str());
      hardware_interface::CommandInterface T1(
                    test.get_name(false),
                    inter,
                    test.get_cmd_interface(inter));
      double c = unif(re);
      T1.set_value(c);
      EXPECT_TRUE(test.get_cmd(i++)==c);

  } 
}

TEST(motor_manager_test,exp_stt_int)
{
  uniform_real_distribution<double> unif(-5.0,5.0);
  default_random_engine re;
  moteus::Pi3HatMoteusInterface::ServoCommand cmd_data;
  std::vector<moteus::Pi3HatMoteusInterface::ServoReply> msr_data;
  Get_Function gets = [] (std::vector<Reply>& replies,int bus,int id,int opt,int& err, int prov_msg ) -> moteus::QueryResultV2
  {
      int i = 0;
    for (const auto& item : replies) 
    {
      i++;
      if(i>prov_msg)
        break;
      if (item.id == id && item.bus == bus)
      { 
        err = 1;
        return item.result; 
      }
    }
    err = 2;
    return {};
  } ;
  Policy_Function pol = [](bool msg_valid, bool msg_complete,Command* cmd_d)
  {


  };
  uint8_t id = 1;
  uint8_t bus = 1;
  std::string name = "Jnt_test";
  double motor_trans = 1.0, sec_enc_trasn = 0.0;
  pi3hat_hw_interface::motor_manager::Motor_Manager test(
          name,
          &cmd_data,
          &msr_data,
          motor_trans,
          sec_enc_trasn,
          id,
          bus,
          pol,
          gets);
  
  std::vector<std::string> int_type = test.get_state_type();
  interface_tpl res;
  //std::printf("PASSO\n");
  int i = 0;
  for(auto inter : int_type)
  {
      EXPECT_TRUE(name == test.get_name(false));
      std::printf("interface %s\n",inter.c_str());
      hardware_interface::StateInterface T1(
                    test.get_name(false),
                    inter,
                    test.get_stt_interface(inter,false));
      double c = unif(re);
      test.set_stt(c,i++);
      //printf("value %lf\n",T1.get_value());
     
      EXPECT_TRUE(T1.get_value()==c);

  }
  // test second ecoder stt interface
  motor_trans = 1.0;
  sec_enc_trasn = 1.0;
  pi3hat_hw_interface::motor_manager::Motor_Manager test1(
          name,
          &cmd_data,
          &msr_data,
          motor_trans,
          sec_enc_trasn,
          id,
          bus,
          pol,
          gets);
  
  int_type = test1.get_state_type();
  i = 0;
  std::string name_cond;
  
  //std::printf("lo stato originale di enc vel e\' %lf\n",test1.get_stt(5));
  for(auto inter : int_type)
  {
      name_cond = i<4?name:(name+"_ENCODER1");
      
      
      std::printf("name1 %s and istance_name %s\n",name_cond.c_str(),test1.get_name(i<4?false:true).c_str());
     try{
      //std::printf("lo stato originale di enc vel e\' %lf\n",test1.get_stt(5));
      EXPECT_TRUE(name_cond == test1.get_name(i<4?false:true));
      }
      catch(std::logic_error err)
      {
        printf("error throwed in get name\n");
      }
      std::printf("interface %s\n",inter.c_str());
      hardware_interface::StateInterface T1(
                    test1.get_name(i<4?false:true),
                    inter,
                    i<4?test1.get_stt_interface(inter,false):
                    test1.get_stt_interface(inter,true));
      double c = unif(re);
      //printf("old value %lf, i %d\n", test1.get_stt(i),i);
      test1.set_stt(c,i++);
      //printf("value %lf and i %d\n",T1.get_value(),i);
      EXPECT_TRUE(T1.get_value() ==c);

  }
}

TEST(motor_manager_test,make_pos)
{
  moteus::PositionResolution T1;
  moteus::QueryCommandV2 U1;
  T1.position = moteus::Resolution::kInt16;
  T1.velocity = moteus::Resolution::kInt16;
  T1.feedforward_torque = moteus::Resolution::kInt16;
  T1.kd_scale = moteus::Resolution::kInt16;
  T1.kp_scale = moteus::Resolution::kInt16;
  T1.maximum_torque = moteus::Resolution::kInt16;
  T1.stop_position = moteus::Resolution::kInt16;
  T1.watchdog_timeout = moteus::Resolution::kInt16;
  U1.position = moteus::Resolution::kInt16;
  U1.velocity = moteus::Resolution::kInt16;
  U1.torque = moteus::Resolution::kInt16;
  U1.sec_enc_pos = moteus::Resolution::kIgnore;
  U1.sec_enc_vel = moteus::Resolution::kIgnore;

uniform_real_distribution<double> unif(-5.0,5.0);
  default_random_engine re;
  moteus::Pi3HatMoteusInterface::ServoCommand cmd_data;
  std::vector<moteus::Pi3HatMoteusInterface::ServoReply> msr_data;
  Get_Function gets = [] (std::vector<Reply>& replies,int bus,int id,int opt,int& err, int prov_msg ) -> moteus::QueryResultV2
  {
      int i = 0;
    for (const auto& item : replies) 
    {
      i++;
      if(i>prov_msg)
        break;
      if (item.id == id && item.bus == bus)
      { 
        err = 1;
        return item.result; 
      }
    }
    err = 2;
    return {};
  } ;
  Policy_Function pol = [](bool msg_valid, bool msg_complete,Command* cmd_d)
  {


  };
  uint8_t id = 1;
  uint8_t bus = 1;
  std::string name = "Jnt_test";
  double motor_trans = 1.0, sec_enc_trasn = 0.0;
  pi3hat_hw_interface::motor_manager::Motor_Manager test(
          name,
          &cmd_data,
          &msr_data,
          motor_trans,
          sec_enc_trasn,
          id,
          bus,
          pol,
          gets);
  
  std::vector<hardware_interface::CommandInterface> interfaces;
  std::vector<double> cmd_val;
  test.set_command_resolution(T1);
  test.set_query_resolution(U1);
  std::vector<std::string> int_type = test.get_command_type();
  int i = 0;
  for(auto inter_t : int_type)
  {
    interfaces.emplace_back(hardware_interface::CommandInterface(
      test.get_name(false),
      inter_t,
      test.get_cmd_interface(inter_t)
    ));
  }
  cmd_val.resize(interfaces.size());
  for(int i = 0; i < interfaces.size(); i++)
  {
      cmd_val[i] = unif(re);
      interfaces[i].set_value(cmd_val[i]);
  }
  
  test.make_position();
  EXPECT_TRUE(cmd_data.id == test.get_id());
  EXPECT_TRUE(cmd_data.bus == test.get_bus());

  EXPECT_TRUE(cmd_data.resolution == T1);
  // std::printf("expected::\n %d,%d,%d,%d,%d\n%d,%d,%d,%d",
  // T1.position,T1.velocity,T1.feedforward_torque,
  // T1.kd_scale,T1.kp_scale,T1.maximum_torque,
  // T1.stop_position,T1.watchdog_timeout);
  // std::printf("getted::\n%d,%d,%d,%d\n%d,%d,%d,%d",
  // cmd_data.resolution.position,cmd_data.resolution.velocity,cmd_data.resolution.feedforward_torque,
  // cmd_data.resolution.kd_scale,cmd_data.resolution.kp_scale,cmd_data.resolution.maximum_torque,
  // cmd_data.resolution.stop_position,cmd_data.resolution.watchdog_timeout);
  auto transm = test.get_transmission();
  EXPECT_TRUE(cmd_data.query == U1);
  EXPECT_TRUE(
          cmd_data.mode == moteus::Mode::kPosition &&
          cmd_data.position.position == cmd_val[0]*transm &&
          cmd_data.position.velocity == cmd_val[1]*transm &&
          cmd_data.position.feedforward_torque == cmd_val[2]/transm &&
          cmd_data.position.kp_scale == cmd_val[3] &&
          cmd_data.position.kd_scale == cmd_val[4]
          );  
  std::printf("Passd all test for make position\n");




}

TEST(motor_manager_test,make_stop)
{
  moteus::PositionResolution T1;
  moteus::QueryCommandV2 U1;
  T1.position = moteus::Resolution::kInt16;
  T1.velocity = moteus::Resolution::kInt16;
  T1.feedforward_torque = moteus::Resolution::kInt16;
  T1.kd_scale = moteus::Resolution::kInt16;
  T1.kp_scale = moteus::Resolution::kInt16;
  T1.maximum_torque = moteus::Resolution::kInt16;
  T1.stop_position = moteus::Resolution::kInt16;
  T1.watchdog_timeout = moteus::Resolution::kInt16;
  U1.position = moteus::Resolution::kInt16;
  U1.velocity = moteus::Resolution::kInt16;
  U1.torque = moteus::Resolution::kInt16;
  U1.sec_enc_pos = moteus::Resolution::kIgnore;
  U1.sec_enc_vel = moteus::Resolution::kIgnore;

uniform_real_distribution<double> unif(-5.0,5.0);
  default_random_engine re;
  moteus::Pi3HatMoteusInterface::ServoCommand cmd_data;
  std::vector<moteus::Pi3HatMoteusInterface::ServoReply> msr_data;
  Get_Function gets = [] (std::vector<Reply>& replies,int bus,int id,int opt,int& err, int prov_msg ) -> moteus::QueryResultV2
  {
      int i = 0;
    for (const auto& item : replies) 
    {
      i++;
      if(i>prov_msg)
        break;
      if (item.id == id && item.bus == bus)
      { 
        err = 1;
        return item.result; 
      }
    }
    err = 2;
    return {};
  } ;
  Policy_Function pol = [](bool msg_valid, bool msg_complete,Command* cmd_d)
  {


  };
  uint8_t id = 1;
  uint8_t bus = 1;
  std::string name = "Jnt_test";
  double motor_trans = 1.0, sec_enc_trasn = 0.0;
  pi3hat_hw_interface::motor_manager::Motor_Manager test(
          name,
          &cmd_data,
          &msr_data,
          motor_trans,
          sec_enc_trasn,
          id,
          bus,
          pol,
          gets);
  
  std::vector<hardware_interface::CommandInterface> interfaces;
  std::vector<double> cmd_val;
  test.set_command_resolution(T1);
  test.set_query_resolution(U1);
  std::vector<std::string> int_type = test.get_command_type();
  int i = 0;
  for(auto inter_t : int_type)
  {
    interfaces.emplace_back(hardware_interface::CommandInterface(
      test.get_name(false),
      inter_t,
      test.get_cmd_interface(inter_t)
    ));
  }
  cmd_val.resize(interfaces.size());
  for(int i = 0; i < interfaces.size(); i++)
  {
      cmd_val[i] = unif(re);
      interfaces[i].set_value(cmd_val[i]);
  }
  
  test.make_stop();
  EXPECT_TRUE(cmd_data.id == test.get_id());
  EXPECT_TRUE(cmd_data.bus == test.get_bus());
  std::printf("expected::\n %d,%d,%d,%d,%d\n%d,%d,%d,%d",
  T1.position,T1.velocity,T1.feedforward_torque,
  T1.kd_scale,T1.kp_scale,T1.maximum_torque,
  T1.stop_position,T1.watchdog_timeout);
  std::printf("getted::\n%d,%d,%d,%d\n%d,%d,%d,%d",
  cmd_data.resolution.position,cmd_data.resolution.velocity,cmd_data.resolution.feedforward_torque,
  cmd_data.resolution.kd_scale,cmd_data.resolution.kp_scale,cmd_data.resolution.maximum_torque,
  cmd_data.resolution.stop_position,cmd_data.resolution.watchdog_timeout);
  EXPECT_TRUE(cmd_data.resolution == T1);
  EXPECT_TRUE(cmd_data.query == U1);
  EXPECT_TRUE(cmd_data.mode == moteus::Mode::kStopped );
  std::printf("Passd all test for make position\n");

}

TEST(motor_manager_test,read_1)
{
  uniform_real_distribution<double> unif(-5.0,5.0);
  default_random_engine re;
  moteus::Pi3HatMoteusInterface::ServoCommand cmd_data;
  std::vector<moteus::Pi3HatMoteusInterface::ServoReply> msr_data;
  Get_Function gets = [] (std::vector<Reply>& replies,int bus,int id,int opt,int& err, int prov_msg ) -> moteus::QueryResultV2
  {
      int i = 0;
    for (const auto& item : replies) 
    {
      i++;
      if(i>prov_msg)
        break;
      if (item.id == id && item.bus == bus)
      { 
        err = 0;
        return item.result; 
        std::printf("FOUND: %d,%d",item.id,item.bus);
      }
    }
    std::printf("NOT FOUND");
    err = 1;
    return {};
  } ;
  Policy_Function pol = [](bool msg_valid, bool msg_complete,Command* cmd_d)
  {


  };
  uint8_t id = 1;
  uint8_t bus = 1;
  std::string name = "Jnt_test";
  double motor_trans = 1.0, sec_enc_trasn = 0.0;
  pi3hat_hw_interface::motor_manager::Motor_Manager test(
          name,
          &cmd_data,
          &msr_data,
          motor_trans,
          sec_enc_trasn,
          id,
          bus,
          pol,
          gets);
  msr_data.resize(NUM_REP_TEST);
  for(int i = 0; i < NUM_REP_TEST; i++)
  {
      msr_data[i].id = i;
      msr_data[i].bus = bus;
      msr_data[i].result.position = unif(re);
      msr_data[i].result.velocity = unif(re);
      msr_data[i].result.torque = unif(re);
      msr_data[i].result.temperature = unif(re);
  }
  std::vector<double> msr_val;
  std::vector<std::string> int_type = test.get_state_type();
  std::vector<hardware_interface::StateInterface> interfaces;
  int i = 0;
  for(auto inter_t : int_type)
  {
    std::printf("name %s type %s\n",test.get_name(false).c_str(),inter_t.c_str());
    interfaces.emplace_back(hardware_interface::StateInterface(
      test.get_name(false),
      inter_t,
      test.get_stt_interface(inter_t,false)
    ));
    
  }

  auto transm = test.get_transmission();
  test.get_motor_state(NUM_REP_TEST);
  std::vector<double> test_res = {
    msr_data[1].result.position/transm,
    msr_data[1].result.velocity/transm,
    msr_data[1].result.torque*transm,
    msr_data[1].result.temperature
  };
  for(int i=0; i< int_type.size();i++)
  {
    std::printf("expected %lf vs getted %lf\n",test_res[i],interfaces[i].get_value());
    EXPECT_TRUE(interfaces[i].get_value() == test_res[i] );
  }
  EXPECT_TRUE(true);
}

TEST(motor_manager_test,read_2)
{
  uniform_real_distribution<double> unif(-5.0,5.0);
  default_random_engine re;
  moteus::Pi3HatMoteusInterface::ServoCommand cmd_data;
  std::vector<moteus::Pi3HatMoteusInterface::ServoReply> msr_data;
  Get_Function gets = [] (std::vector<Reply>& replies,int bus,int id,int opt,int& err, int prov_msg ) -> moteus::QueryResultV2
  {
      int i = 0;
    for (const auto& item : replies) 
    {
      i++;
      if(i>prov_msg)
        break;
      if (item.id == id && item.bus == bus)
      { 
        err = 0;
        return item.result; 
        std::printf("FOUND: %d,%d",item.id,item.bus);
      }
    }
    std::printf("NOT FOUND");
    err = 1;
    return {};
  } ;
  Policy_Function pol = [](bool msg_valid, bool msg_complete,Command* cmd_d)
  {


  };
  uint8_t id = 1;
  uint8_t bus = 1;
  std::string name = "Jnt_test";
  double motor_trans = 1.0, sec_enc_trasn = 1.0;
  pi3hat_hw_interface::motor_manager::Motor_Manager test(
          name,
          &cmd_data,
          &msr_data,
          motor_trans,
          sec_enc_trasn,
          id,
          bus,
          pol,
          gets);
  msr_data.resize(NUM_REP_TEST);
  for(int i = 0; i < NUM_REP_TEST; i++)
  {
      msr_data[i].id = i;
      msr_data[i].bus = bus;
      msr_data[i].result.position = unif(re);
      msr_data[i].result.velocity = unif(re);
      msr_data[i].result.torque = unif(re);
      msr_data[i].result.temperature = unif(re);
      msr_data[i].result.sec_enc_pos = unif(re);
      msr_data[i].result.sec_enc_vel = unif(re);
  }
  std::vector<double> msr_val;
  std::vector<std::string> int_type = test.get_state_type();
  std::vector<hardware_interface::StateInterface> interfaces;
  int i = 0;
  for(auto inter_t : int_type)
  {
    std::printf("name %s type %s\n",test.get_name(i<4?false:true).c_str(),inter_t.c_str());
    interfaces.emplace_back(hardware_interface::StateInterface(
      test.get_name(i<4?false:true),
      inter_t,
      test.get_stt_interface(inter_t,i<4?false:true)
    ));
    i++;
    
  }

  auto transm = test.get_transmission();
  auto s_transm = test.get_sec_transmission();
  test.get_motor_state(NUM_REP_TEST);
  std::vector<double> test_res = {
    msr_data[1].result.position/transm,
    msr_data[1].result.velocity/transm,
    msr_data[1].result.torque*transm,
    msr_data[1].result.temperature,
    msr_data[1].result.sec_enc_pos/s_transm,
    msr_data[1].result.sec_enc_vel/s_transm
  };
  for(int i=0; i< int_type.size();i++)
  {
    std::printf("expected %lf vs getted %lf\n",test_res[i],interfaces[i].get_value());
    EXPECT_TRUE(interfaces[i].get_value() == test_res[i] );
  }
  EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
