
/**
 * publish a custom message called vector
 */

#include "rclcpp/rclcpp.hpp"
#include "demo_topic/msg/vector.hpp"

#include <functional>
#include <chrono>

typedef demo_topic::msg::Vector Vector;

const double X_DEFAULT_VAL = 3.0;
const double Y_DEFAULT_VAL = 4.0;

class VecPubNode: public rclcpp::Node
{
	public:
		VecPubNode(): Node("vec_pub_node")
		{
			vector_ = Vector();
			vector_.x = X_DEFAULT_VAL;
			vector_.y = Y_DEFAULT_VAL;
			publisher_ = this->create_publisher<Vector>("vector", 10);
			timer_ = this->create_wall_timer(
				std::chrono::seconds(1), 
				std::bind(&VecPubNode::pub_vec_callback, this)
			);
		}
	private:
		rclcpp::Publisher<Vector>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
		Vector vector_;

		void pub_vec_callback() 
		{
			publisher_->publish(vector_);
		}
};

int main(int argc, char* argv[]) 
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VecPubNode>());
	rclcpp::shutdown();
	return 0;
}