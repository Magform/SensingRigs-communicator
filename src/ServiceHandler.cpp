#include "ServiceHandler.h"

ServiceHandler::ServiceHandler(int id, FileReader fileReader) : Node("sensingrigs_communicator"),
    _id(id),
    _fileReader(fileReader)
{
    _service = this->create_service<sensingrigs_communicator::srv::DataQuery>(
        "sensingrigs_communicator",
        std::bind(&ServiceHandler::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "Service ready: sensingrigs_communicator");
}


void ServiceHandler::handle_request(
    const std::shared_ptr<sensingrigs_communicator::srv::DataQuery::Request> request,
    std::shared_ptr<sensingrigs_communicator::srv::DataQuery::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Request received with id: %.2f", request->id);

    if(request->id == _id){
        getDataFromFile(file_name_, response->irdata, response->odmdata, response->strdata, response->status);
    }
}