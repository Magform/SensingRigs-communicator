#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono>
#include "sensingrigs_communicator/srv/data_query.hpp"

using namespace std::chrono_literals;


class DesertClient : public rclcpp::Node
{
    public:
    DesertClient() : Node("desert_client")
    {
        RCLCPP_INFO(this -> get_logger(), "Client Created");
        client = this -> create_client<sensingrigs_communicator::srv::DataQuery>("sensingrigs_communicator");

    }
    int request( float id){
        auto request = std::make_shared<sensingrigs_communicator::srv::DataQuery::Request>();
        request ->id = id;
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return 0;
            }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = client->async_send_request(request);
        rclcpp::spin_until_future_complete(this -> get_node_base_interface(), result);
        RCLCPP_INFO(this-> get_logger(), "RESULT ARRIVED");
        auto response = result.get();
        RCLCPP_INFO(this->get_logger(), "Data received:");
        for (const auto & mono : response->mono_data) {
            RCLCPP_INFO(this->get_logger(), "Mono label: %s", mono.label.c_str());
        }
        return 1;
    }
    private:
    rclcpp::Client<sensingrigs_communicator::srv::DataQuery>::SharedPtr client;

};
int main(int argc, char * argv[]){
    // setenv("RMW_IMPLEMENTATION", "rmw_desert", 1);
    // setenv("DESERT_PORT", "5000", 1);
    rclcpp::init(argc, argv);
    auto client_noode = std::make_shared<DesertClient>();
    client_noode ->request(1);
    rclcpp::spin(client_noode);
    rclcpp::shutdown();
    return 0;

}