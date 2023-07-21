#include "pi3hat_hw_interface/moteus_pi3hat_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

using namespace rclcpp;
namespace pi3hat_hw_interface
{
    namespace moteus_pi3hat_interface
    {
        MoteusPi3Hat_Interface::MoteusPi3Hat_Interface()
        {
            RCLCPP_INFO(get_logger("PINO"),"Passo");
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

        std::vector<hardware_interface::StateInterface> MoteusPi3Hat_Interface::export_state_interfaces()
        {
            return {};
        };
        std::vector<hardware_interface::CommandInterface> MoteusPi3Hat_Interface::export_command_interfaces()
        {
            return {};
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