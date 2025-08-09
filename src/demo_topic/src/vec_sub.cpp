#include "rclcpp/rclcpp.hpp"
#include "demo_topic/msg/vector.hpp"

#include <functional>
#include <iostream>
#include <cmath>

const double THETA_DEFAULT_DEGREE = 0.0;

typedef demo_topic::msg::Vector Vector;

class VecSubNode: public rclcpp::Node
{
	public:
		VecSubNode(): Node("vec_sub_node")
		{
			this->declare_parameter<double>("theta_degree_val", THETA_DEFAULT_DEGREE);
			publisher_ = this->create_publisher<Vector>("rotated_vector", 10);
			subscription_ = this->create_subscription<Vector>(
				"vector", 10, 
				std::bind(&VecSubNode::rotate_vector_callback, this, std::placeholders::_1)
			);
			std::cout << "Rotating vector...\n";
		}
	private:
		rclcpp::Subscription<Vector>::SharedPtr subscription_;
		rclcpp::Publisher<Vector>::SharedPtr publisher_;
		
		void rotate_vector_callback(const Vector& vector)
		{
			auto rotated_vector = Vector();
			double radian = this->get_parameter("theta_degree_val").as_double() * M_PI / 180.0;
			double s = std::sin(radian), c = std::cos(radian);
			rotated_vector.x = vector.x*c - vector.y*s;
			rotated_vector.y = vector.x*s + vector.y*c;
			publisher_->publish(rotated_vector);
		}
};

int main(int argc, char *argv[])
{   
  rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VecSubNode>());
	rclcpp::shutdown();
  return 0;
}
