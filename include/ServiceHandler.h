#IFNDEF SERVICE_HANDLER_H
#DEFINE SERVICE_HANDLER_H

#include "rclcpp/rclcpp.hpp"
#include "sensingrigs_communicator/srv/data_query.hpp"
#include <functional>


class ServiceHandler : public rclcpp::Node{
    
public:
    using DataCallback = std::function<void(
        std::vector<sensingrigs_communicator::msg::MonoIR>&,
        std::vector<sensingrigs_communicator::msg::Odometry>&,
        std::vector<sensingrigs_communicator::msg::StereoID>&,
        int32_t&
    )>;

    ServiceHandler(int id, DataCallback data_callback);

private:
    rclcpp::Service<sensingrigs_communicator::srv::DataQuery>::SharedPtr _service;

    int _id;
    DataCallback _data_callback;

    void handle_request(
        const std::shared_ptr<sensingrigs_communicator::srv::DataQuery::Request> request,
        std::shared_ptr<sensingrigs_communicator::srv::DataQuery::Response> response);
};

#ENDIF // SERVICE_HANDLER_H