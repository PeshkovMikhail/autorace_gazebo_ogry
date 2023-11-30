#include <functional>
#include <memory>
#include <thread>

#include "autorace_communication_gazebo_ogry/action/intersection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "autorace_core_gazebo_ogry/intersection_visibility.h"

namespace intersection_action_cpp
{
class IntersectionActionServer : public rclcpp::Node
{
public:
  using Intersection = autorace_communication_gazebo_ogry::action::Intersection;
  using GoalHandleIntersection = rclcpp_action::ServerGoalHandle<Intersection>;

  INTERSECTION_ACTION_CPP_PUBLIC
  explicit IntersectionActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("intersection_action_server", options)
  {
    using namespace std::placeholders;

    lidar_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&IntersectionActionServer::update_lidar, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("/robot/odom", 10, std::bind(&IntersectionActionServer::update_odom, this, std::placeholders::_1));

    vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    this->action_server_ = rclcpp_action::create_server<Intersection>(
      this,
      "intersection",
      std::bind(&IntersectionActionServer::handle_goal, this, _1, _2),
      std::bind(&IntersectionActionServer::handle_cancel, this, _1),
      std::bind(&IntersectionActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Intersection>::SharedPtr action_server_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Intersection::Goal> goal)
  {
    // RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleIntersection> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleIntersection> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&IntersectionActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleIntersection> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Intersection::Feedback>();

    RCLCPP_INFO(get_logger(), "INTERSECTION TASK STARTED");
    // auto & sequence = feedback->partial_sequence;
    // sequence.push_back(0);
    // sequence.push_back(1);
    auto result = std::make_shared<Intersection::Result>();

    // for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    //   // Check if there is a cancel request
    //   if (goal_handle->is_canceling()) {
    //     result->sequence = sequence;
    //     goal_handle->canceled(result);
    //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //     return;
    //   }
    //   // Update sequence
    //   sequence.push_back(sequence[i] + sequence[i - 1]);
    //   // Publish feedback
    //   goal_handle->publish_feedback(feedback);
    //   RCLCPP_INFO(this->get_logger(), "Publish feedback");

    //   loop_rate.sleep();
    // }

    // // Check if goal is done
    if (rclcpp::ok()) {
      result->finished = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "INTERSECTION TASK FINISHED");
    }
  }


  void update_lidar(const sensor_msgs::msg::LaserScan &msg) {

  }

  void update_odom(const nav_msgs::msg::Odometry &msg) {

  }
};  // class IntersectionActionServer

}  // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(intersection_action_cpp::IntersectionActionServer)