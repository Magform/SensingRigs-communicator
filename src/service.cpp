#include "rclcpp/rclcpp.hpp"
#include "ServiceHandler.h"
#include "FileReader.h"

#define ID 1
#define file_name "test.txt"

void getDataFromFileCallback(
    std::vector<sensingrigs_communicator::msg::MonoIR>& mono_data,
    std::vector<sensingrigs_communicator::msg::StereoIR>& stereo_data,
    std::vector<sensingrigs_communicator::msg::Odometry>& odom_data,
    int32_t& status){
        
    // getDataFromFile(file_name, mono_data, stereo_data, odom_data, status);
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto data_callback = getDataFromFileCallback;

    auto service_node = std::make_shared<ServiceHandler>(ID, data_callback);
    rclcpp::spin(service_node);

    rclcpp::shutdown();
    return 0;
}
