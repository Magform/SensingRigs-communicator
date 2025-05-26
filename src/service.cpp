#include "rclcpp/rclcpp.hpp"
#include "ServiceHandler.h"
#include "FileReader.h"

#define ID 1
#define file_name test.txt

void getDataFromFileCallback(
    std::vector<sensingrigs_communicator::msg::MonoIR>& irdata,
    std::vector<sensingrigs_communicator::msg::Odometry>& odmdata,
    std::vector<sensingrigs_communicator::msg::StereoID>& strdata,
    int32_t& status){
        
    getDataFromFile(file_name, irdata, odmdata, strdata, status);
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto data_callback = getDataFromFileCallback;

    auto service_node = std::make_shared<ServiceHandler>(ID, data_callback);
    rclcpp::spin(service_node);

    rclcpp::shutdown();
    return 0;
}
