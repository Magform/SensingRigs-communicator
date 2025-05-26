#include "rclcpp/rclcpp.hpp"
#include "ServiceHandler.h"
#include "FileReader.h"

#define ID 1
#define file_name data_file.txt

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    FileReader file_reader(file_name);

    auto service_node = std::make_shared<ServiceHandler>(ID, file_reader);
    rclcpp::spin(service_node);

    rclcpp::shutdown();
    return 0;
}
