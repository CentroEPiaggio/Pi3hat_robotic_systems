#include "pi3hat_hw_interface/moteus_pi3hat_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#define LOGGER_NAME "MoteusPi3Hat_Interface"
#define CPU 1
using namespace rclcpp;
namespace pi3hat_hw_interface
{
    namespace moteus_pi3hat_interface
    {

         
        MoteusPi3Hat_Interface::MoteusPi3Hat_Interface()//:
       // communication_thread_(opt_thread_.set_cpu(CPU))
        {
            // create the comunnication thread_
            //communication_thread_ = new MoteusInterface(opt);
            gets_ = [] (std::vector<Reply>& replies,int bus,int id,int opt,int& err, int prov_msg ) -> moteus::QueryResultV2
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

            poly_ = [](bool msg_valid, bool msg_complete,Command* cmd_d)
            {


            };
        };
        MoteusPi3Hat_Interface::~MoteusPi3Hat_Interface()
        {

        };

        CallbackReturn MoteusPi3Hat_Interface::on_init(const hardware_interface::HardwareInfo & info)
        {
            // get jnt number and resize my structure
            auto n_jnt = info.joints.size();
             RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"joint number is %ld",
                n_jnt);
            cmd_data_.resize(n_jnt);
            msr_data_.resize(2*n_jnt);
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
                        &prova,
                        &msr_data_,
                        std::stod(joint.parameters.at("motor_transmission")),
                        std::stod(joint.parameters.at("sec_enc_transmission")),
                        id,
                        bus,
                        poly_,
                        gets_
                    );
                motors_.push_back(motor);
                    
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
            // for(int i = 0; i < NUM_STOP; i++)
            // {
            //     for(auto motor : motors_)
            //     {
            //         motor.make_stop();
            //     }   
            //     cycle();
            //     while(!can_recvd_.valid())
            //     {
            //         rclcpp::sleep_for(SLEEP_FOR_10MS);
            //     }
            //     // eventually check fault
            // }
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_deactivate(const rclcpp_lifecycle::State&)
        {
            // for(int i = 0; i < NUM_STOP; i++)
            // {
            //     for(auto motor : motors_)
            //     {
            //         motor.make_stop();
            //     }   
            //     cycle();
            //     while(!can_recvd_.valid())
            //     {
            //         rclcpp::sleep_for(SLEEP_FOR_10MS);
            //     }
            //     // eventually check fault
            // }
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_shutdown(const rclcpp_lifecycle::State&)
        {
            // communication_thread_->~Pi3HatMoteusInterface();
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
            if(
                //can_recvd_.valid()
                true)
            {
                for(auto motor: motors_)
                    motor.set_msg_valid(false);
            }
            else
            {
                //out = can_recvd_.get();
                out.query_result_size = msr_data_.size();
                for(auto motor : motors_)
                {
                    motor.set_msg_valid(true);
                    motor.get_motor_state(out.query_result_size);
                }

            }
            return hardware_interface::return_type::OK;
        };

        hardware_interface::return_type MoteusPi3Hat_Interface::write(const rclcpp::Time & , const rclcpp::Duration & ) 
        {
            for(auto motor : motors_)
                motor.make_position();
            cycle();
            return hardware_interface::return_type::OK;
        };


    }
}PLUGINLIB_EXPORT_CLASS(
  pi3hat_hw_interface::moteus_pi3hat_interface::MoteusPi3Hat_Interface, hardware_interface::SystemInterface)