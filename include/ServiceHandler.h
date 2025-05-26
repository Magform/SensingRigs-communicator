#IFNDEF SERVICE_HANDLER_H
#DEFINE SERVICE_HANDLER_H

#include "rclcpp/rclcpp.hpp"
#include "sensingrigs_communicator/srv/data_query.hpp"
#include "FileReader.h"


class ServiceHandler : public rclcpp::Node
{
public:
    ServiceHandler(int id, FileReader fileReader);

private:
    rclcpp::Service<sensingrigs_communicator::srv::DataQuery>::SharedPtr _service;

    int _id;
    FileReader _fileReader;

    void handle_request(
        const std::shared_ptr<sensingrigs_communicator::srv::DataQuery::Request> request,
        std::shared_ptr<sensingrigs_communicator::srv::DataQuery::Response> response);
};

#ENDIF // SERVICE_HANDLER_H