#include "pi3hat_hw_interface/moteus_pi3hat_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#define LOGGER_NAME "MoteusPi3Hat_Interface"
#define CPU 1
using namespace rclcpp;
#define MAX_COUNTER 10000
namespace pi3hat_hw_interface
{
    namespace moteus_pi3hat_interface
    {

         
        MoteusPi3Hat_Interface::MoteusPi3Hat_Interface():
        communication_thread_(opt_thread_.set_cpu(CPU))
        {
            // create the comunnication thread_
            //communication_thread_ = new MoteusInterface(opt);
            gets_ = [] (std::vector<Reply>& replies,int bus,int id,int opt,int& err, int prov_msg ) -> moteus::QueryResultV2
            {
                int i = 0;
                bool discard = prov_msg == 1 ? false:true;
                // RCLCPP_WARN(
                //         rclcpp::get_logger(LOGGER_NAME),
                //         "The message will be discarad %d"
                //         ,discard);
                for (const auto& item : replies) 
                {
                    if(item.id != std::nan("1") && item.bus != std::nan("1"))
                    {

                        if (item.id == id && item.bus == bus )
                        { 
                            if(discard)
                            {
                                discard = false;
                            }
                            else
                            {
                                err = 0;
                                // RCLCPP_INFO(rclcpp::get_logger("AA"),"the motor id %d and bus %d has pkt index %d",id,bus,i);
                                return item.result; 
                                //std::printf("FOUND: %d,%d",item.id,item.bus);
                            }
                        }
                    }
                }
                //std::printf("NOT FOUND");
                err = 1;
                for(auto rep : replies)
                {
                    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "The %d-th repies belong to ID: %d and bus %d the measured pos is %lf, vel is %lf, trq is %lf and temperature is %lf",i,rep.id,rep.bus, rep.result.position,
                    rep.result.velocity, rep.result.torque, rep.result.temperature);
                    i++;
                }
                return {};
            };

            poly_ = [](bool msg_valid, bool msg_complete,Command* cmd_d)
            {


            };
            
        };
        MoteusPi3Hat_Interface::~MoteusPi3Hat_Interface()
        {
            auto a = std::chrono::nanoseconds(10000000);
            for(int i = 0; i < NUM_STOP; i++)
            {

                while(!can_recvd_.valid())
                {
                    rclcpp::sleep_for(a);
                }
                
                for(auto motor : motors_)
                {
                    motor.make_stop();
                }  
                
                // RCLCPP_INFO(rclcpp::get_logger("LOGGER_NAME"),"Stop command %d",i);

                // rclcpp::sleep_for(a);
                // RCLCPP_INFO(rclcpp::get_logger("LOGGER_NAME"),"Stop command %d",i);

                cycle();
                
                // eventually check fault
            }
            communication_thread_.~Pi3HatMoteusInterface();
        };

        CallbackReturn MoteusPi3Hat_Interface::on_init(const hardware_interface::HardwareInfo & info)
        {
            // get jnt number and resize my structure
            auto n_jnt = info.joints.size();
             RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"joint number is %ld",
                n_jnt);
            cmd_data_.resize(n_jnt);
            msr_data_.resize(2*n_jnt);
            pkt_loss_.resize(n_jnt);
            motors_.resize(0);
            Command prova;
            uint8_t bus,id;
            Motor_Manager motor;
            int i=0;
            for(auto joint :info.joints)
            {
                id = static_cast<uint8_t>(std::stoi(joint.parameters.at("id")));
                
                bus =static_cast<uint8_t>(std::stoi(joint.parameters.at("bus")));
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"insert joint name %s and [id,bus] :: [%d,%d]",
                joint.name.c_str(),id,bus);
                motor.set_all(
                        joint.name,
                        &cmd_data_[i],
                        &msr_data_,
                        std::stod(joint.parameters.at("motor_transmission")),
                        std::stod(joint.parameters.at("sec_enc_transmission")),
                        id,
                        bus,
                        poly_,
                        gets_
                    );
                motors_.push_back(motor);

                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"the setted id is %d",motor.get_id());
                    
                i++;
            }
            i = 0;
            for(auto motor: motors_)
            {
                 RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"%d insert joint name %s and [id,bus] :: [%d,%d]",
                i,motor.get_name(false).c_str(),motor.get_id(),motor.get_bus());
                i++;
            }
            if(i != n_jnt)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Error in association of joints to motor structure");
                return CallbackReturn::ERROR;
            }
            data_.commands = {cmd_data_.data(),cmd_data_.size()};
            data_.replies = {msr_data_.data(),msr_data_.size()};
             RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"INIT COMPLETED");
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_configure(const rclcpp_lifecycle::State& )
        {
            // set command and query resolution
            moteus::PositionResolution PC;
            PC.position = moteus::Resolution::kFloat;
            PC.velocity = moteus::Resolution::kFloat;
            PC.feedforward_torque = moteus::Resolution::kFloat;
            PC.kp_scale = moteus::Resolution::kInt16;
            PC.kd_scale = moteus::Resolution::kInt16;
            

            moteus::QueryCommandV2 PQ;
            PQ.position = moteus::Resolution::kFloat;
            PQ.velocity = moteus::Resolution::kFloat;
            PQ.torque = moteus::Resolution::kFloat;
            PQ.temperature = moteus::Resolution::kFloat;
            PQ.fault = moteus::Resolution::kInt8;

            for(auto motor : motors_)
            {
                motor.set_command_resolution(PC);
                if(motor.get_sec_transmission() == 0.0)
                {
                    PQ.sec_enc_pos = moteus::Resolution::kIgnore;
                    PQ.sec_enc_vel = moteus::Resolution::kIgnore;
                }
                else
                {
                    PQ.sec_enc_pos = moteus::Resolution::kFloat;
                    PQ.sec_enc_vel = moteus::Resolution::kFloat;
                }
                motor.set_query_resolution(PQ);
            } 


            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_cleanup(const rclcpp_lifecycle::State&)
        {
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_activate(const rclcpp_lifecycle::State&)
        {
            auto a = std::chrono::nanoseconds(10000000);

            for(int i = 0; i < NUM_STOP; i++)
            {
                // for(int i = 0; i < cmd_data_.size(); i++)
                // {
                //     cmd_data_[i].id = 9;
                // }
                for(auto motor : motors_)
                {
                    motor.make_stop();
                    //RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "the motor id is %d",motor.get_id());
                }   
                
                // for(int i = 0; i < cmd_data_.size(); i++)
                // {                   
                //     RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "%d the pos and vel data are %lf and %lf",i,cmd_data_[i].position.position,cmd_data_[i].position.velocity);
                //     RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "%d the id and bus data are %d and %d",i,cmd_data_[i].id,cmd_data_[i].bus);
                //     RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "%d the kp_scale and kd_scale data are %lf and %lf",i,cmd_data_[i].position.kp_scale,cmd_data_[i].position.kd_scale);
                // }
                rclcpp::sleep_for(a);
                try
                {
                    cycle();
                }
                
                catch(std::logic_error)
                {
                    assert(false);
                }

                RCLCPP_INFO(rclcpp::get_logger("LOGGER_NAME"),"stop msg %d",i);
                rclcpp::sleep_for(a);
                while(!can_recvd_.valid())
                {

                    RCLCPP_INFO(rclcpp::get_logger("LOGGER_NAME"),"await stop %d",can_recvd_.valid());

                    rclcpp::sleep_for(a);
                }
                // RCLCPP_INFO(rclcpp::get_logger("LOGGER_NAME"),"no await stop %d",can_recvd_.valid());

                // eventually check fault
            }
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_deactivate(const rclcpp_lifecycle::State&)
        {
            auto a = std::chrono::nanoseconds(10000000);
            for(int i = 0; i < NUM_STOP; i++)
            {
                for(auto motor : motors_)
                {
                    motor.make_stop();
                }  
                
                // RCLCPP_INFO(rclcpp::get_logger("LOGGER_NAME"),"Stop command %d",i);

                // rclcpp::sleep_for(a);
                // RCLCPP_INFO(rclcpp::get_logger("LOGGER_NAME"),"Stop command %d",i);

                cycle();
                while(!can_recvd_.valid())
                {
                    rclcpp::sleep_for(a);
                }
                // eventually check fault
            }
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_shutdown(const rclcpp_lifecycle::State&)
        {
            communication_thread_.~Pi3HatMoteusInterface();
            return CallbackReturn::SUCCESS;
        };

        std::vector<hardware_interface::StateInterface> MoteusPi3Hat_Interface::export_state_interfaces()
        {   
            std::vector<hardware_interface::StateInterface> stt_int;
            std::vector<std::string> int_type;
            int i;
            for(auto motor : motors_)
            {
                // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"insert joint name %s and [id,bus] :: [%d,%d]",
                // motor.get_name(false).c_str(),motor.get_id(),motor.get_bus());
                i = 0;
                int_type = motor.get_state_type();
                for(auto type : int_type)
                {
                    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"insert joint name %s type %s and [id,bus] :: [%d,%d]",
                motor.get_name(false).c_str(),type.c_str(),motor.get_id(),motor.get_bus());
                    try
                    {
                    stt_int.emplace_back(
                        motor.get_name(i<4?false:true),
                        type,
                        motor.get_stt_interface(type,i<4?false:true)
                    );
                    }
                    catch(std::logic_error err)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Raised error during the state interface exporting ID:%d bus:%d",motor.get_id(),motor.get_bus());
                        assert(false);
                    }
                    i++;
                }
            }  
            return stt_int;
        };
        
        std::vector<hardware_interface::CommandInterface> MoteusPi3Hat_Interface::export_command_interfaces()
        {
           std::vector<hardware_interface::CommandInterface> cmd_int;
            std::vector<std::string> int_type;
            for(auto motor : motors_)
            {
                int_type = motor.get_command_type();
                for(auto type : int_type)
                {
                    try
                    {
                    cmd_int.emplace_back(
                        motor.get_name(false),
                        type,
                        motor.get_cmd_interface(type)
                    );
                    }
                    catch(std::logic_error err)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Raised error during the state interface exporting ID:%d bus:%d",motor.get_id(),motor.get_bus());
                        assert(false);
                    }
                }
            }
            return cmd_int;
        };

        hardware_interface::return_type MoteusPi3Hat_Interface::read(const rclcpp::Time & , const rclcpp::Duration & ) 
        {
            Output out;
            double perc;
            int i = 0;
            count_ ++;
            // if( count_ %1 00 == 0)
            //     RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "100");
            
            if(can_recvd_.wait_for(10us) != std::future_status::ready)
            {
                not_val_cycle_++;
                
                for(auto motor: motors_)
                    motor.set_msg_valid(false);
                // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Not valid msg");
                valid_ = false;
            }
            else
            {
                out = can_recvd_.get();
                // if(out.query_result_size < 2)
                // {
                //     for(auto rep : msr_data_)
                //     {
                //         RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "The %d-th repies belong to ID: %d and bus %d the measured pos is %lf, vel is %lf, trq is %lf and temperature is %lf",i,rep.id,rep.bus, rep.result.position,
                //         rep.result.velocity, rep.result.torque, rep.result.temperature);
                //         i++;
                //     }
                // }
                valid_ = true;
                
                //RCLCPP_INFO(rclcpp::get_logger(LOGGER_Nout = can_recvd_.get();AME), "the output num is  %ld",out.query_result_size);
                // out.query_result_size = msr_data_.size();
                for(auto &motor : motors_)
                {
                    motor.set_msg_valid(true);
                    // RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "not complete msg motor %d is  %d",motor.get_id(),motor.get_msg_arrived());
                    i = motor.get_motor_state(motor.get_msg_arrived());
                    if(i > 10)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Raised error %d",i);
                        return hardware_interface::return_type::ERROR;
                    }
                    // motor.print_pl();
                }

            }
            
            if(count_ % MAX_COUNT == 0)
            {
                int lost_pkt ;
                for(size_t j = 0; j < pkt_loss_.size(); j++)
                {

                    perc = pkt_loss_.at(j) * MAX_COUNT * epoch_count_;
                    // RCLCPP_WARN(
                    //     rclcpp::get_logger(LOGGER_NAME),
                    //     "motor %d has percentage of %lf"
                    //     ,motors_[j].get_id(),perc);
                    epoch_count_++;
                    lost_pkt = motors_[j].get_pkg_loss();
                    motors_[j].reset_pkg_loss();

                    // RCLCPP_WARN(
                    //     rclcpp::get_logger(LOGGER_NAME),
                    //     "motor %d has no cpmpleted %d  msgs "
                    //     ,motors_[j].get_id(),a)
                    
                    pkt_loss_.at(j) = static_cast<double>(perc +lost_pkt)/static_cast<double>(MAX_COUNT*epoch_count_);
                    // RCLCPP_WARN(
                    //     rclcpp::get_logger(LOGGER_NAME),
                    //     "motor %d has percentage of %lf after update"
                    //     ,motors_[j].get_id(),perc);
                }
                //RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Not valid msg percentage is %d/ %d",not_val_cycle_,count_);
                
                perc = valid_loss_ * MAX_COUNT * (epoch_count_ -1);

                valid_loss_ =  (perc + static_cast<double>(not_val_cycle_))/static_cast<double>(epoch_count_*MAX_COUNT);
                
               // RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Not valid msg count is %d",not_val_cycle_);
                count_ = 0;
                not_val_cycle_ = 0;
                RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),"the validity loss is %lf",valid_loss_);
                i = 0;
                for(auto d : pkt_loss_)
                {
                    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),"the package loss  of motor %d is %lf",motors_[i].get_id(),d);
                    i++;
                }
            }
            return hardware_interface::return_type::OK;
        };

        hardware_interface::return_type MoteusPi3Hat_Interface::write(const rclcpp::Time & , const rclcpp::Duration & ) 
        {
            if(valid_)
            {
                for(auto &motor : motors_)
                {
                    motor.make_position();
                    // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "the motor ID is %d",motor.get_id());

                }

                // int i  = 0;
                // for(auto cmd : cmd_data_)
                // {
                    
                //     //RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "The %d-th command belong to ID: %d and bus %d the commanded pos is %lf, vel is %lf, trq is %lf and kp,kd  is %lf,%lf",i,cmd.id,cmd.bus, cmd.position.position,
                //     //cmd.position.velocity, cmd.position.feedforward_torque, cmd.position.kp_scale,cmd.position.kd_scale);
                //     //i++;
                   // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"the mode is ");
                // }
                
                try
                {
                    cycle();
                }
                catch(std::logic_error err)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Pi3Hat cycle error has rised");
                    assert(false);
                }
            }
            return hardware_interface::return_type::OK;
        };


    }
}PLUGINLIB_EXPORT_CLASS(
  pi3hat_hw_interface::moteus_pi3hat_interface::MoteusPi3Hat_Interface, hardware_interface::SystemInterface)