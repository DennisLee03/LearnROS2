#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "demo_action/action/fibonacci.hpp"

#include <iostream>

typedef demo_action::action::Fibonacci FibonacciAction;
typedef rclcpp_action::ServerGoalHandle<FibonacciAction> GoalHandle;

class FibActionServerNode: public rclcpp::Node
{
  public:
		FibActionServerNode(): Node("fib_action_server_node")
		{
			server_ = rclcpp_action::create_server<FibonacciAction>(
				this, "fibonacci",
				std::bind(&FibActionServerNode::handle_goal, this, 
					std::placeholders::_1, std::placeholders::_2),
				std::bind(&FibActionServerNode::handle_cancel, this,
					std::placeholders::_1),
				std::bind(&FibActionServerNode::handle_accepted, this,
					std::placeholders::_1)
			);
			std::cout << "Fibonacci Action Server Started\n";
		}
	private:
		rclcpp_action::GoalResponse handle_goal(
			const rclcpp_action::GoalUUID& uuid,
			std::shared_ptr<const FibonacciAction::Goal> goal) // pointer to constant content, it means the content of goal is unmodifiable
		{
			(void)uuid;
			std::cout << "Received goal n = " << goal->n << ", to calculate fib(n)\n";
			/**
			 * Enumerator:
			 * ACCEPT_AND_EXECUTE, ACCEPT_AND_DEFER, REJECT
			 */
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}

		rclcpp_action::CancelResponse handle_cancel(
			const std::shared_ptr<GoalHandle> goal_handle) // constant pointer, it means goal_handle cannot point to other space
		{
			(void)goal_handle;
			std::cout << "Recieved request to cancel goal\n";
			return rclcpp_action::CancelResponse::ACCEPT;
		}

		void handle_accepted(
			const std::shared_ptr<GoalHandle> goal_handle)
		{
			std::thread{std::bind(&FibActionServerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
		}

		void execute(
			const std::shared_ptr<GoalHandle> goal_handle)
		{
			std::cout << "Execute goal\n";

			// 3 components of an action:
			const auto goal = goal_handle->get_goal();
			auto feedback = std::make_shared<FibonacciAction::Feedback>();
			auto result = std::make_shared<FibonacciAction::Result>();

			fib_seq[0] = 0;
			fib_seq[1] = 1;

			/* 2 hz */
			rclcpp::Rate rate(2);
			for(int i=0; i<=goal->n; i++)
			{
				if(i==0 || i==1)
				{
					feedback->fib_i = fib_seq[i];
					goal_handle->publish_feedback(feedback);
					continue;
				}

				fib_seq[i] = fib_seq[i-1] + fib_seq[i-2];
				feedback->fib_i = fib_seq[i];
				goal_handle->publish_feedback(feedback);
				
				rate.sleep();
			}
			/* 2 hz*/

			result->fib_n = fib_seq[goal->n];
			goal_handle->succeed(result);
			std::cout << "Goal Succeeded\n";
		}

		rclcpp_action::Server<FibonacciAction>::SharedPtr server_;
		int fib_seq[1000] = {0};
};

int main(int argc, char const *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FibActionServerNode>());
	rclcpp::shutdown();
	return 0;
}