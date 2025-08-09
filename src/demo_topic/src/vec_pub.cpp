
/**
 * publish a custom message called vector
 */

#include "rclcpp/rclcpp.hpp"
#include "demo_topic/msg/vector.hpp"

#include <functional>
#include <iostream>
#include <chrono>

typedef demo_topic::msg::Vector Vector;

const double X_DEFAULT_VAL = 3.0;
const double Y_DEFAULT_VAL = 4.0;

class VecPubNode: public rclcpp::Node
{
	public:
		VecPubNode(): Node("vec_pub_node")
		{
			this->declare_parameter<double>("x_val", X_DEFAULT_VAL);
			this->declare_parameter<double>("y_val", Y_DEFAULT_VAL);
			publisher_ = this->create_publisher<Vector>("vector", 10);
			timer_ = this->create_wall_timer(
				std::chrono::seconds(1), 
				std::bind(&VecPubNode::pub_vec_callback, this)
			);
			std::cout << "Publishing vector...\n";		
		}
	private:
		rclcpp::Publisher<Vector>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;

		void pub_vec_callback() 
		{
			auto vector = Vector();
			vector.x = this->get_parameter("x_val").as_double();
			vector.y = this->get_parameter("y_val").as_double();
			publisher_->publish(vector);
		}
};

int main(int argc, char* argv[]) 
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VecPubNode>());
	rclcpp::shutdown();
	return 0;
}