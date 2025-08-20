#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

const double DEFAULT_GOAL_X = 5.0;
const double DEFAULT_GOAL_Y = 5.0;

class SimpleGoToGoalNode: public rclcpp::Node
{
    public:
        SimpleGoToGoalNode(): Node("simple_go_to_goal_node")
        {
            // parameters declaration
            this->declare_parameter<double>("goal_x", DEFAULT_GOAL_X);
            this->declare_parameter<double>("goal_y", DEFAULT_GOAL_Y);

            subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10, 
                std::bind(&SimpleGoToGoalNode::odom_callback, std::placeholders::_1)
            );

            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            goal_x_ = this->get_parameter("goal_x").as_double();
            goal_y_ = this->get_parameter("goal_y").as_double();
            RCLCPP_INFO(this->get_logger(), "Goal set to: (.2f, .2f)", goal_x_, goal_y_);
        }

    private:

        // using proportional control
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            // fetch current goal by parameters
            goal_x_ = this->get_parameter("goal_x").as_double();
            goal_y_ = this->get_parameter("goal_y").as_double();

            
        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        double goal_x_, goal_y_;
        double current_x_, current_y_, current_yaw_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleGoToGoalNode>());
    rclcpp::shutdown();
    return 0;
}