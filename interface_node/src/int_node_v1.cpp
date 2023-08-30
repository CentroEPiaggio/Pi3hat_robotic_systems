#include <stdio.h>
#include "rclcpp/node.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "moteus_pi3hat/moteus_protocol.h"
#include "moteus_pi3hat/pi3hat_moteus_interface.h"
#include "moteus_pi3hat/realtime.h"
#include <future>
#include <string>
#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/exceptions/exceptions.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <mutex>





using namespace mjbots;
using namespace std::chrono_literals;
using std::placeholders::_1;
using MoteusInterface = moteus::Pi3HatMoteusInterface;
using Command = moteus::Pi3HatMoteusInterface::ServoCommand;
using Reply = moteus::Pi3HatMoteusInterface::ServoReply;
using Options = moteus::Pi3HatMoteusInterface::Options;
using Data = moteus::Pi3HatMoteusInterface::Data;
using Output = moteus::Pi3HatMoteusInterface::Output;
using CmdMsg = std_msgs::msg::Float32MultiArray;

namespace pi3hat_node
{

    class Pi3Hat_int : public rclcpp::Node
    {
            
            public:
                Pi3Hat_int(int id, int bus, int kp_s, int kd_s, Options trd_opt):
                    Node("interface_node"),
                    first_msg_(true),
                    id_(id),
                    bus_(bus),
                    mot_pos_(0),
                    mot_vel_(0),
                    mot_pos_cmd_(0),
                    mot_vel_cmd_(0),
                    k_p_scale_(kp_s),
                    k_d_scale_(kd_s),
                    count_(0),
                    comm_interface_(MoteusInterface())
                {
                    
                    // setup command and query resolution 
                    mot_read_.resize(5);
                    mot_comd_.resize(1);

                    Initialize();

                    data_.commands = {mot_comd_.data(),mot_comd_.size()};
                    data_.replies = {mot_read_.data(),mot_read_.size()};

                    auto a = data_.commands[0];
                    RCLCPP_INFO(this->get_logger(),"the length are %f,%f",a.position.position,a.position.velocity);
                    // create substriber 
                    // create timer
                    timer_ = this->create_wall_timer(100us,std::bind(&Pi3Hat_int::timer_callback, this));
                    sub_ = this->create_subscription<CmdMsg>("MJbot_prova",10,std::bind(&Pi3Hat_int::sub_callback,this,_1));
                    RCLCPP_INFO(this->get_logger(),"Completed the node creation");
                }
                ~Pi3Hat_int()
                {   
                    auto a = std::chrono::nanoseconds(10000000);
                    while(!can_rcvd_.valid())
                    {
                        rclcpp::sleep_for(a);
                    }
                    RCLCPP_INFO(this->get_logger(),"Wait to stop the system");
                    for(int i=0;i<10;i++)
                    {
                        // send stop position 
                        Fill_Command(0);
                        const auto promise = std::make_shared<std::promise<Output>>();
                            //send position command message with cycle 
                            comm_interface_.Cycle(
                                data_,
                                [promise](const Output& out)
                                {
                                    promise->set_value(out);
                                }
                            );
                        while(!can_rcvd_.valid())
                        {
                            rclcpp::sleep_for(a);
                        }
                        RCLCPP_INFO(this->get_logger(),"send stop command number %d",i);
                    }
                    comm_interface_.~Pi3HatMoteusInterface();
                    RCLCPP_INFO(this->get_logger(),"Descructor has been called\n");


                }
                void Initialize()
                {
                    moteus::PositionResolution res;
                    res.position = moteus::Resolution::kFloat;
                    res.velocity = moteus::Resolution::kFloat;
                    res.feedforward_torque = moteus::Resolution::kFloat;
                    res.kp_scale = moteus::Resolution::kInt16;
                    res.kd_scale = moteus::Resolution::kInt16;
                    res.maximum_torque = moteus::Resolution::kIgnore;
                    res.stop_position = moteus::Resolution::kIgnore;
                    res.watchdog_timeout = moteus::Resolution::kIgnore;
                    int i = 0;
                    for (auto& cmd : mot_comd_) {
                    cmd.resolution = res;
                    RCLCPP_INFO(this->get_logger(),"is %d",i);
                    }
                }
                void Fill_Command(int cmd)
                {
                    auto& cmd_msg = mot_comd_.at(0);
                    cmd_msg.id = id_;
                    cmd_msg.bus = bus_;
                    if(cmd==0)
                        cmd_msg.mode = moteus::Mode::kStopped;

                    else
                    {
                        std::lock_guard<std::mutex> guard(mux);
                        cmd_msg.mode = moteus::Mode::kPosition;
                        
                        cmd_msg.position.position = mot_pos_cmd_;
                        cmd_msg.position.velocity = mot_vel_cmd_;
                        cmd_msg.position.kp_scale = k_p_scale_;
                        cmd_msg.position.kd_scale = k_d_scale_;
                        cmd_msg.position.feedforward_torque = 0.0;
                        //RCLCPP_INFO(this->get_logger(),"command value are %f %f",cmd_msg.position.position,cmd_msg.position.velocity);
                    }
                }
                void sub_callback(const CmdMsg::SharedPtr msg)
                {
                    // subscriber callback to change the desired trajectory 
                    std::lock_guard<std::mutex> guard(mux);
                    mot_pos_cmd_ = msg->data[0];
                    mot_vel_cmd_ = msg->data[1];
                }
                void timer_callback()
                {
                    //RCLCPP_INFO(this->get_logger(),"timer callback");
                    if(count_ < 10)
                    {
                        Fill_Command(0);
                        if (can_rcvd_.valid())
                        {
                            data_.commands = {mot_comd_.data(),mot_comd_.size()};
                            data_.replies = {mot_read_.data(),mot_read_.size()};   
                            const auto promise = std::make_shared<std::promise<Output>>();
                            //send position command message with cycle 
                            comm_interface_.Cycle(
                                data_,
                                [promise](const Output& out)
                                {
                                    promise->set_value(out);
                                }
                            );
                            can_rcvd_ = promise->get_future();
                        }
                    }
                    else
                    {
                        // if the old message is valid or the flag first msg is true
                        if (can_rcvd_.valid() || first_msg_)
                        {
                            if(first_msg_)
                                first_msg_ = false;
                            else
                            {
                                // try to read 
                                const auto rcv_out = can_rcvd_.get();
                                if(rcv_out.query_result_size > 0)
                                {
                                    if(rcv_out.query_result_size > 1)
                                    {
                                        count_dup_ ++;
                                        RCLCPP_WARN(this->get_logger(),"count is %d",count_dup_);
                                    }
                                    RCLCPP_WARN(this->get_logger(),"Data correctly managed by the interface");
                                    const auto read_res = this->Get(mot_read_,id_,bus_);

                                    // update internal structure 
                                    mot_pos_ = read_res.position;
                                    mot_vel_ = read_res.velocity;
                                    // RCLCPP_INFO(this->get_logger(),
                                    // "the read value of the second ecoder is %f",read_res.sec_enc_pos);
                                    
                                //    RCLCPP_INFO(this->get_logger(),
                                //     "the readed position and velocity are %f and %f",mot_pos_,mot_vel_);
                                }

                                else
                                // roswarining and go to the next cycle
                                    RCLCPP_WARN(this->get_logger(),
                                    "Driver measure are not arrived and count is %d",++ n_a_rsp_);
                            }

                        }
                        // fill the command and reply and data structure
                        Fill_Command(1);
                        
                        auto a = data_.commands[0];
                        // if(a.mode == moteus::Mode::kPosition)
                        //     RCLCPP_INFO(this->get_logger(),"command position %d",count_);
                     
                        const auto promise = std::make_shared<std::promise<Output>>();
                        //send position command message with cycle 
                        comm_interface_.Cycle(
                            data_,
                            [promise](const Output& out)
                            {
                                promise->set_value(out);
                            }
                        );
                        can_rcvd_ = promise->get_future();
                        
                        RCLCPP_INFO(this->get_logger(),"valid output %d",can_rcvd_.valid());
                         
                        
                    }
                    count_ ++;
                    
                }
                moteus::QueryResultV2 Get(const std::vector<MoteusInterface::ServoReply>& replies, int id, int bus) 
                {
                    for (const auto& item : replies) 
                    {
                    if (item.id == id && item.bus == bus) { return item.result; }
                    }
                    return {};
                }

            private:
                bool first_msg_;
                int id_;
                int bus_;

                double mot_pos_,mot_vel_,mot_pos_cmd_,mot_vel_cmd_,k_p_scale_,k_d_scale_;
                int count_,n_a_rsp_ = 0, count_dup_ = 0;
                std::future<MoteusInterface::Output> can_res;
                std::vector<Reply> mot_read_;
                std::vector<Command> mot_comd_;
                Options opt_;
                Data data_;
                std::future<Output> can_rcvd_;
                MoteusInterface comm_interface_;
                
                std::mutex mux;

                rclcpp::TimerBase::SharedPtr timer_;
                rclcpp::Subscription<CmdMsg>::SharedPtr sub_;


                

    };

}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  std::shared_ptr<pi3hat_node::Pi3Hat_int> node;
  Options pi3hat_opt;
  pi3hat_opt.cpu = 1;
  node = std::make_shared<pi3hat_node::Pi3Hat_int>(4,1,10,10,pi3hat_opt);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}