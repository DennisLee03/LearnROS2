#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "demo_action/action/fibonacci.hpp"

#include <iostream>

typedef demo_action::action::Fibonacci FibonacciAction;
typedef rclcpp_action::ClientGoalHandle<FibonacciAction> GoalHandle;

class FibActionClinetNode: public rclcpp::Node
{
	public:
		FibActionClinetNode(): Node("fib_action_client_node")
		{
			client_ = rclcpp_action::create_client<FibonacciAction>(this, "fibonacci");
			prompt_user_for_goal();
		}
	private:
		void prompt_user_for_goal()
		{
			using std::placeholders::_1;
			using std::placeholders::_2;
			auto goal_msg = FibonacciAction::Goal();

			// prompt
			std::cout << "Enter a postive integer to find fib(n): ";
			std::cin >> goal_msg.n;

			this->client_->wait_for_action_server();
			std::cout << "Sending Goal\n";

			auto send_goal_options = rclcpp_action::Client<FibonacciAction>::SendGoalOptions();
			send_goal_options.goal_response_callback = 
				std::bind(&FibActionClinetNode::goal_response_callback, this, _1);
			send_goal_options.feedback_callback = 
				std::bind(&FibActionClinetNode::feedback_callback, this, _1, _2);
			send_goal_options.result_callback = 
				std::bind(&FibActionClinetNode::result_callback, this, _1);

			this->client_->async_send_goal(goal_msg, send_goal_options);
		}

		void goal_response_callback(GoalHandle::SharedPtr future)
		{
			auto goal_handle = future.get();
			if(!goal_handle)
			{
				std::cout << "Goal was rejected by the server.\n";
			} else {
				std::cout << "Goal was accepted by the server.\n";
			}
		}

		void feedback_callback(
			GoalHandle::SharedPtr future,
			const std::shared_ptr<const FibonacciAction::Feedback> feedback)
		{
			(void)future;
			std::cout << "Feedback: " << feedback->fib_i << "\n";
		}

		void result_callback(const GoalHandle::WrappedResult& result)
		{
			switch (result.code)
			{
			case rclcpp_action::ResultCode::SUCCEEDED:
				std::cout << "Result: fib(n) = " << result.result->fib_n << "\n";
				break;
			case rclcpp_action::ResultCode::ABORTED:
				std::cout << "Goal was aborted\n";
				break;
			case rclcpp_action::ResultCode::CANCELED:
				std::cout << "Goal was canceled\n";
				break;
			default:
				std::cout << "Unknown result code\n";
				break;
			}
			rclcpp::shutdown();
		}
		rclcpp_action::Client<FibonacciAction>::SharedPtr client_;
};

int main(int argc, char const *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FibActionClinetNode>());
	rclcpp::shutdown();
	return 0;
}
