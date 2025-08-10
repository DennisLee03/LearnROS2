#include "rclcpp/rclcpp.hpp"
#include "demo_service/srv/vector.hpp"

#include <iostream>

typedef demo_service::srv::Vector VectorSrv;



int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);

	auto client_node = rclcpp::Node::make_shared("vec_client_node");
	auto client = client_node->create_client<VectorSrv>("calculate_length");

	auto request = std::make_shared<VectorSrv::Request>();
	std::cout << "Please type a vector to calculate its length: ";
	std::cin >> request->x >> request->y;

	client->wait_for_service();
	auto response = client->async_send_request(request);

	if(rclcpp::spin_until_future_complete(client_node, response) ==
		rclcpp::FutureReturnCode::SUCCESS)
	{
		std::cout << "Vector Length = " << response.get()->length << "\n";
	} else {
		std::cout << "There was an error processing the request...\n";
	}

	rclcpp::shutdown();
	return 0;
}