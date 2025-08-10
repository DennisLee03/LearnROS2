#include "rclcpp/rclcpp.hpp"
#include "demo_service/srv/vector.hpp"

#include <functional>
#include <iostream>

typedef demo_service::srv::Vector VectroSrv;

class VecServerNode: public rclcpp::Node
{
	public:
		VecServerNode(): Node("vec_server_node")
		{
			server_ = this->create_service<VectroSrv>(
				"calculate_length", 
				std::bind(&VecServerNode::calculate_length_callback, this, std::placeholders::_1, std::placeholders::_2)
			);
			std::cout << "Vector Service is running...\n";
		}
	private:
		rclcpp::Service<VectroSrv>::SharedPtr server_;
		void calculate_length_callback(
			const VectroSrv::Request::SharedPtr request, 
			VectroSrv::Response::SharedPtr response)
		{
			double x = request->x, y = request->y;
			response->length = std::sqrt(x*x + y*y);	
		}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VecServerNode>());
	rclcpp::shutdown();
	return 0;
}