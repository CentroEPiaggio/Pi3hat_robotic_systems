#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <memory>
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#define FB_LINK_SOLO "solo12::base_link"
#define FB_LINK_ANYM "anymal_c::base"
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pi3hat_moteus_int_msgs/msg/packet_pass.hpp"

using std::placeholders::_1;


class Record_Benchmark : public rclcpp::Node
{

    public:
        Record_Benchmark():
        Node("BenchmarkRecorder")
        {
            RCLCPP_INFO(get_logger(),"Start Node");
            topic_name_ = std::string("state_broadcaster/performance_indexes");
            declare_parameter("duration_s",600);
            declare_parameter("path","ros_bag");
            rclcpp::QoS out_qos(10);
            try
            {
                writer_ = std::make_unique<rosbag2_cpp::Writer>();
                bag_folder_path_ = get_parameter("path").as_string();
                RCLCPP_INFO(get_logger(),"Start writer: %s", bag_folder_path_.c_str());
                const rosbag2_storage::StorageOptions strg_opt({bag_folder_path_,"sqlite3"});
                // const rosbag2_storage::StorageOptions strg_opt({bag_folder_path_,"mcap"});
                writer_->open(strg_opt);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(),"Error opening bag: %s", e.what());
                assert(true);
            }
                
            RCLCPP_INFO(get_logger(),"Create Writer");
            writer_->create_topic(
                    {topic_name_,
                    "pi3hat_moteus_int_msgs/msg/PacketPass",
                    rmw_get_serialization_format(),
                    ""}
                );
            sub_ = create_subscription<pi3hat_moteus_int_msgs::msg::PacketPass>(topic_name_,out_qos,
                std::bind(&Record_Benchmark::sub_callback,this,_1));

        };
        ~Record_Benchmark(){};

        void sub_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
            rclcpp::Time time_stamp = this->now();
            const std::string name = topic_name_;

            const std::string type = "pi3hat_moteus_int_msgs/msg/PacketPass";

            writer_-> write(msg,name,type,time_stamp);
        };
    private:

    std::shared_ptr<rclcpp::Subscription<pi3hat_moteus_int_msgs::msg::PacketPass>> sub_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    std::string bag_folder_path_, topic_name_; 


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Record_Benchmark>());
    
    rclcpp::shutdown();
    return 0;
}