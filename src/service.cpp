#include "rclcpp/rclcpp.hpp"
#include "ServiceHandler.h"
#include "FileReader.h"

#define ID 1
#define FILE_NAME "data.txt"

FileReader file_reader(FILE_NAME);

void getDataFromFileCallback(
    std::vector<sensingrigs_communicator::msg::MonoIR>& mono_data,
    std::vector<sensingrigs_communicator::msg::StereoIR>& stereo_data,
    std::vector<sensingrigs_communicator::msg::Odometry>& odom_data,
    int32_t& status,
    const builtin_interfaces::msg::Time& start_time,
    const builtin_interfaces::msg::Time& end_time)
{

    file_reader.getDataFromFile(mono_data, stereo_data, odom_data, status, start_time, end_time);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto data_callback = getDataFromFileCallback;

    auto service_node = std::make_shared<ServiceHandler>(ID, data_callback);
    rclcpp::spin(service_node);

    rclcpp::shutdown();
    return 0;
}
