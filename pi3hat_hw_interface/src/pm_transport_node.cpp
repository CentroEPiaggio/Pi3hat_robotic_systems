// moteus include 
#include "moteus_pi3hat/moteus.h"
#include "moteus_pi3hat/pi3hat_moteus_transport.h"
// ros2 include
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "std_msgs/msg/int16.hpp"
// GP include 
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <chrono>
#include <future>
#include <functional>
#include <stdio.h>

using namespace std::chrono_literals;
using namespace rclcpp;
using Transport_Options = mjbots::pi3hat::Pi3HatMoteusTransport::Options;
using Transport = mjbots::pi3hat::Pi3HatMoteusTransport;
using Attitude = mjbots::pi3hat::Attitude;
using Controller = mjbots::moteus::Controller;
using Controller_Option = Controller::Options;
class AsyncCallback
{
    public:
        mjbots::moteus::CompletionCallback callback()
        {
            return [&](int v)
            {
                std::unique_lock<std::mutex> lock(mutex_);
                RCLCPP_DEBUG(rclcpp::get_logger("CALLBACK"),"completition callback");
                done_.store(true);
                result_.store(v,std::memory_order_release);
            };
        };
        int try_consume()
        {
            //preliminary check to not direct block the mutex
            if(!done_.load(std::memory_order_acquire))
                return -1;
                // lock the mutex in the actual scope
                std::lock_guard<std::mutex> lock(mutex_);
                done_.store(false);
                RCLCPP_DEBUG(rclcpp::get_logger("CALLBACK"),"try consume");

                return result_.load();
            
        }
     private:
        std::atomic<bool> done_{false};
        std::atomic<int> result_{0};
        std::mutex mutex_;
};
class MoteusPi3hatNode : public  Node
{
    public:
        MoteusPi3hatNode(const Transport_Options opt_t, int motor_id):
        Node("MP_node")
        {
            t_ = std::make_shared<Transport>(opt_t);
            Controller_Option opt_c;
            opt_c.id = motor_id;
            opt_c.transport = t_;
            opt_c.query_format.extra[0].register_number = mjbots::moteus::Register::kControlPositionError;
            opt_c.query_format.extra[0].resolution = mjbots::moteus::Resolution::kFloat;
            opt_c.query_format.extra[1].register_number = mjbots::moteus::Register::kControlVelocityError;
            opt_c.query_format.extra[1].resolution = mjbots::moteus::Resolution::kFloat;
            c_ = std::make_shared<Controller>(opt_c);
            publisher_ = this->create_publisher<std_msgs::msg::Int16>("logging",200);
            pub_count_ = this->create_publisher<std_msgs::msg::Int16>("logging_count",200);
            
        }
        ~MoteusPi3hatNode()
        {

            c_->DiagnosticWrite("d stop \n");
            RCLCPP_INFO(this->get_logger(),"tel stop");
            // conf_set("conf set servo.pid_position.kd 0.0\n");
            t_.reset();
            c_.reset();
        }
        void conf_set(std::string message)
        {

            std::string res;
            c_->DiagnosticWrite("tel stop \n");
            RCLCPP_INFO(this->get_logger(),"tel stop");
            c_->DiagnosticFlush();
            RCLCPP_INFO(this->get_logger(),"tel stop");
            c_->DiagnosticCommand(message);

            res = c_->DiagnosticCommand("conf get servo.pid_position.kp",mjbots::moteus::Controller::kExpectSingleLine);
            RCLCPP_INFO(this->get_logger(),"the result is %s", res.c_str());
        }
        void send_sync_vel(const int count=100)
        {
            mjbots::moteus::PositionMode::Command cmd;
            cmd.position = std::numeric_limits<float>::quiet_NaN();
            cmd.velocity = 2.0;
            std::vector<mjbots::moteus::CanFdFrame> frames,replies;
            
            frames.push_back(c_->MakePosition(cmd));
            for(int i =0; i< count; i++)
            {
                {
                mjbots::moteus::BlockingCallback b_clb;
                RCLCPP_INFO(this->get_logger(),"");
                t_->Cycle(
                    frames.data(),frames.size(), &replies, &att_,nullptr,nullptr,b_clb.callback()
                );
                int result = b_clb.Wait();
                RCLCPP_INFO(this->get_logger(), "pos %d",result);
                }
                // t_->BlockingCycle(frames.data(),frames.size(),replies);
                for(auto rep : replies)
                {
                    if(rep.source==4)
                    {
                        auto res = mjbots::moteus::Query::Parse(rep.data,rep.size);

                        RCLCPP_INFO(this->get_logger(), "pos %f::::vel %f, %d",res.position,res.velocity,i);
                    }
                }
                // sleep_for(5ms);
                 RCLCPP_INFO(this->get_logger(),"");
            }
            frames.clear();
            frames.push_back(c_->MakeStop());
            t_->Cycle(
                    frames.data(),frames.size(), &replies, &att_,nullptr,nullptr,[&](int )
                    {
                        RCLCPP_INFO(this->get_logger(),"end cycle");
                    }
                );
            RCLCPP_INFO(this->get_logger(), "end cycle");


        }
        void start_control(float vel)
        {
            this->cmd_struct_.position = std::numeric_limits<float>::quiet_NaN();
            this->cmd_struct_.velocity = 1.0;
            
            auto cmd_frame = c_->MakePosition(cmd_struct_);
            cmd_frames_.push_back(cmd_frame);
            timer_ = this->create_wall_timer(
                2ms,
                std::bind(&MoteusPi3hatNode::timer_cbk,this)
            );
        }
        void test_encoder_validity()
        {
            std::vector<mjbots::moteus::CanFdFrame> frames,replies;
            frames.resize(1);
            mjbots::moteus::Query::Format qf;
            
            qf.extra[0].register_number = mjbots::moteus::Register::kEncoder1Position;
            qf.extra[0].resolution = mjbots::moteus::Resolution::kFloat;
            qf.extra[0].register_number = mjbots::moteus::Register::kEncoder1Velocity;
            qf.extra[0].resolution = mjbots::moteus::Resolution::kFloat;
            frames[0] = c_->MakeQuery(&qf);
            auto result = c_->SetQuery(&qf);
            // {
            //     mjbots::moteus::BlockingCallback b_clb;
            //     RCLCPP_INFO(this->get_logger(),"");
            //     t_->Cycle(
            //         frames.data(),frames.size(), &replies, &att_,nullptr,nullptr,b_clb.callback()
            //     );
            //     int result = b_clb.Wait();
                
            //     RCLCPP_INFO(this->get_logger(), "pos %d",result);
            // }
            
            if(result.has_value())
            {
                
               
                
                u_int32_t test = static_cast<u_int32_t>(result->values.extra[0].value);
                RCLCPP_INFO(this->get_logger(),"validity %s",((test & (1<<0)) != 0) && ((test & (1<<1)) != 0) ? "true":"false");
                RCLCPP_INFO(this->get_logger(),"validity %s",((test & (1<<2) != 0)) && ((test & (1<<3)) != 0) != 0 ? "true":"false");
                RCLCPP_INFO(this->get_logger(),"validity %s",((test & (1<<4) != 0)) && ((test & (1<<5)) != 0) != 0 ? "true":"false");
                
            }
            else
                RCLCPP_WARN(this->get_logger(),"no reply received");


        }
        void stop_control()
        {
            cmd_frames_.clear();
            timer_->cancel();
        }
        void timer_cbk()
        {
            std_msgs::msg::Int16 msg = std_msgs::msg::Int16();
            msg.data = 0;
            publisher_->publish(msg);
            msg.data = count;
            pub_count_->publish(msg);
            if(count != 0)
            {
                int res = clb_as_.try_consume();
                if(res == -1)
                {
                    msg.data = 3 ;
                    publisher_->publish(msg);
                    msg.data = count;
                    pub_count_->publish(msg);
                }
                else if(res != 0)
                    RCLCPP_ERROR(this->get_logger(),"ERROR: %d",res);
                
                else
                {
                    msg.data = 1;
                    publisher_->publish(msg);
                    for(auto &rep : replies_)
                    {
                        if(rep.source==4)
                        {
                            auto res = mjbots::moteus::Query::Parse(rep.data,rep.size);

                            RCLCPP_INFO(this->get_logger(), "pos %f::::vel %f, %d",res.extra[0].value,res.extra[1].value,count);
                        }
                    }
                    t_->Cycle(cmd_frames_.data(),cmd_frames_.size(),&replies_,&att_,nullptr,nullptr,clb_as_.callback());
                    
                    msg.data = 2;
                    publisher_->publish(msg);
                    msg.data = count;
                    pub_count_->publish(msg);
                    count++;
                    if(count > 1000)
                        this->stop_control();
                }
            }
            else
            {
                // RCLCPP_INFO(this->get_logger(),"first cycle");
                t_->Cycle(cmd_frames_.data(),cmd_frames_.size(),&replies_,&att_,nullptr,nullptr,clb_as_.callback());
                msg.data = 2;
                publisher_->publish(msg);
                msg.data = count;
                pub_count_->publish(msg);
                count++;
            }
            
        }
    private:
        std::shared_ptr<Transport> t_;
        std::shared_ptr<Controller> c_;
        rclcpp::TimerBase::SharedPtr timer_;
        Attitude att_;
        std::vector<mjbots::moteus::CanFdFrame> cmd_frames_;
        mjbots::moteus::PositionMode::Command cmd_struct_;
        AsyncCallback clb_as_;
        std::vector<mjbots::moteus::CanFdFrame> replies_;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_,pub_count_;
        int count=0;
        float pos=0.0;
        

        
        
};
int main(int argc, char * argv[])
{
    Transport_Options opt_t;
   
    int motor_id = 4;
    int motor_bus = 1;
    opt_t.servo_map[motor_id] = motor_bus;
    opt_t.attitude_rate_hz = 100;
    opt_t.mounting_deg.pitch = 0;
    opt_t.mounting_deg.roll = 0;
    opt_t.mounting_deg.yaw = 0;
    init(argc, argv);
    auto node = std::make_shared<MoteusPi3hatNode>(opt_t,motor_id);
    RCLCPP_INFO(rclcpp::get_logger("pp"),"try to send diagnostic command");
    node->conf_set("d exact 0.0\n");
    node->conf_set("conf set servo.pid_position.kp 0.0\n");
    
    node->conf_set("conf set servo.pid_position.kd 0.0\n");
    RCLCPP_INFO(rclcpp::get_logger("PORCODIO"),"pass here");
    // node->test_encoder_validity();
    node->start_control(5.0);
    // node->send_sync_vel(1000);
    // node->conf_set("conf get servo.pid_position.kp");

    spin(node);

    shutdown();
    
    return 0;
}