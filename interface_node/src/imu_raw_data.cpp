#include "rclcpp/rclcpp.hpp"
#include "moteus_pi3hat/pi3hat.h"
#include <chrono>
#include <vector>

struct DeviceImuRawData
{
    uint16_t present=0;
    float gx = 0.0;
    float gy = 0.0;
    float gz = 0.0;
    float ax = 0.0;
    float ay = 0.0;
    float az = 0.0;

} __attribute__((packed));

struct ImuRawData
{
    mjbots::pi3hat::Point3D gyro;
    mjbots::pi3hat::Point3D accel;
};

class Imu_Raw_Data : public rclcpp::Node
{
    public:
        Imu_Raw_Data():
        Node("imu_row_data")
        {
            mjbots::pi3hat::Pi3Hat::Configuration conf;
            pi3hat_ = new mjbots::pi3hat::Pi3Hat(conf);
            getImuData(true);

            RCLCPP_INFO(get_logger(),"the imu acc is [%f,%f,%f]",raw_data.ax,raw_data.ay,raw_data.az);
            
        }

        void getImuData(bool wait)
        {
            raw_data = {};
            // if (wait) 
            // // {
            //     char buf[2] = {};
            //     do {
                //     pi3hat_->ReadSpi(0,96,buf,sizeof(buf));
                // RCLCPP_INFO(get_logger(),"ddd %d,%d",buf[0],buf[1]);
            //         RCLCPP_INFO(get_logger(),"pass here");
            //         //.Read(0, 96, buf, sizeof(buf));
            //         if (buf[1] == 1) { break; }
            //         // If we spam the STM32 too hard, then it doesn't have any
            //         // cycles left to actually work on the IMU.
            //         std::chrono::nanoseconds ns(20*1000);
            //         rclcpp::sleep_for(ns);
            //         // pi3hat_-> BusyWaitUs(20);
            //     } while (true);
            // }
            do
            { 
                pi3hat_->ReadSpi(2,33,
                reinterpret_cast<char*>(&raw_data)
                ,26);
                // RCLCPP_INFO(get_logger(),"pass here %d",raw_data.present & 0x01 );
            } while (wait && ((raw_data.present & 0x01) == 0));


        }
    private:
        mjbots::pi3hat::Pi3Hat* pi3hat_;
        DeviceImuRawData raw_data;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Imu_Raw_Data>());
  rclcpp::shutdown();
  return 0;
}