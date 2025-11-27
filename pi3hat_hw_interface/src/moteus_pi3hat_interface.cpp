#include "pi3hat_hw_interface/moteus_pi3hat_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

#define LOGGER_NAME "MoteusPi3Hat_Interface"
#define CPU 1
using namespace rclcpp;
#define G 9.81
namespace pi3hat_hw_interface
{
    namespace moteus_pi3hat_interface
    {

         
        MoteusPi3Hat_Interface::MoteusPi3Hat_Interface():
        
        {
            
        };
        MoteusPi3Hat_Interface::~MoteusPi3Hat_Interface()
        {
             
           
        };

        CallbackReturn MoteusPi3Hat_Interface::on_init(const hardware_interface::HardwareInfo & info)
        {
            
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_configure(const rclcpp_lifecycle::State& )
        {
            


            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_cleanup(const rclcpp_lifecycle::State&)
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Pass cleanup");

            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_activate(const rclcpp_lifecycle::State&)
        {

           
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_deactivate(const rclcpp_lifecycle::State&)
        {
            
            return CallbackReturn::SUCCESS;
        };
        
        CallbackReturn MoteusPi3Hat_Interface::on_shutdown(const rclcpp_lifecycle::State&)
        {
            
            return CallbackReturn::SUCCESS;
        };

         CallbackReturn MoteusPi3Hat_Interface::on_error(const rclcpp_lifecycle::State&)
        {
            
            return CallbackReturn::SUCCESS;
        };

        std::vector<hardware_interface::StateInterface> MoteusPi3Hat_Interface::export_state_interfaces()
        {   
            std::vector<hardware_interface::StateInterface> stt_int;
            return stt_int;
        };
        
        std::vector<hardware_interface::CommandInterface> MoteusPi3Hat_Interface::export_command_interfaces()
        {
           std::vector<hardware_interface::CommandInterface> cmd_int;
            std::vector<std::string> int_type;
            for(auto &motor : motors_)
            {
                int_type = motor.get_command_type();
                for(auto &type : int_type)
                {
                    try
                    {
                    cmd_int.emplace_back(
                        motor.get_name(false),
                        type,
                        motor.get_cmd_interface(type)
                    );
                    }
                    catch(std::logic_error &err)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Raised error %s during the state interface exporting ID:%d bus:%d",err.what(),motor.get_id(),motor.get_bus());
                        assert(false);
                    }
                }
            }
            return cmd_int;
        };

        hardware_interface::return_type MoteusPi3Hat_Interface::read(const rclcpp::Time & , const rclcpp::Duration & ) 
        {
            return hardware_interface::return_type::OK;
        };

        hardware_interface::return_type MoteusPi3Hat_Interface::write(const rclcpp::Time & , const rclcpp::Duration & ) 
        {
            return hardware_interface::return_type::OK;
        };


    }
}PLUGINLIB_EXPORT_CLASS(
  pi3hat_hw_interface::moteus_pi3hat_interface::MoteusPi3Hat_Interface, hardware_interface::SystemInterface)
