#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "autorace_communication_gazebo_ogry/action/parking.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "autorace_core_gazebo_ogry/parking_visibility.h"

float calcMSE(float arr1[2], float arr2[2]) {
  float x = arr1[0] - arr2[0];
  x *= x;
  float y = arr1[1] - arr2[1];
  y *= y;
  return (x+y)/2;
}

namespace missions_action_cpp
{
class ParkingActionServer : public rclcpp::Node
{
public:
  using Parking = autorace_communication_gazebo_ogry::action::Parking;
  using GoalHandleParking = rclcpp_action::ServerGoalHandle<Parking>;

  PARKING_ACTION_CPP_PUBLIC
  explicit ParkingActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("parking_action_server", options)
  {
    using namespace std::placeholders;

    lidar_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ParkingActionServer::update_lidar, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&ParkingActionServer::update_odom, this, std::placeholders::_1));

    vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    driver_state_ = create_publisher<std_msgs::msg::Bool>("/driver_state", 10);
    
    this->action_server_ = rclcpp_action::create_server<Parking>(
      this,
      "parking",
      std::bind(&ParkingActionServer::handle_goal, this, _1, _2),
      std::bind(&ParkingActionServer::handle_cancel, this, _1),
      std::bind(&ParkingActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Parking>::SharedPtr action_server_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr driver_state_;

  float current_pose[2] = {0, 0};
  float goal_pose[2] = {-0.29f, 3.51f};
  float finish_pose[2] = {-0.11, 0.98};
  float z_angle;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Parking::Goal> goal)
  {
    // RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleParking> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleParking> goal_handle)
  {
    using namespace std::placeholders;
    // // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ParkingActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleParking> goal_handle)
  {
    rclcpp::Rate loop_rate(50);
    RCLCPP_INFO(get_logger(), "PARKING TASK STARTED");
    
    while(calcMSE(current_pose, goal_pose) > 0.01) {
      loop_rate.sleep();
      //RCLCPP_INFO(get_logger(), "%f", calcMSE(current_pose, goal_pose));
    }
    auto driver_state_msg = std_msgs::msg::Bool();
    auto twist = geometry_msgs::msg::Twist();
    
    
    driver_state_msg.data = false;
    driver_state_->publish(driver_state_msg);

    turn_to_angle(-1.57075, loop_rate);

    twist.linear.x = 0.05;
    vel_publisher_->publish(twist);
    driver_state_msg.data = true;
    driver_state_->publish(driver_state_msg);

    // Check if goal is done
    while(calcMSE(current_pose, finish_pose) > 0.01) {
      loop_rate.sleep();
      //RCLCPP_INFO(get_logger(), "%f", calcMSE(current_pose, goal_pose));
    }
    driver_state_msg.data = false;
    driver_state_->publish(driver_state_msg);

    turn_to_angle(3.14, loop_rate);

    // driver_state_msg.data = true;
    // driver_state_->publish(driver_state_msg);


    
    success:
    auto result = std::make_shared<Parking::Result>();
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "PARKING TASK FINISHED");
    }
  }


  void update_lidar(const sensor_msgs::msg::LaserScan &msg) {
    // msg.ranges
  }

  void turn_to_angle(float angle, rclcpp::Rate &loop_rate) {
    auto twist = geometry_msgs::msg::Twist();
    if (z_angle > angle)
      twist.angular.z = -0.5;
    else
      twist.angular.z = 0.5;

    vel_publisher_->publish(twist);
    while(std::fabs(z_angle - angle) > 0.01) {
      loop_rate.sleep();
    }
    twist.angular.z=0;
    vel_publisher_->publish(twist);
  }

  void update_odom(const nav_msgs::msg::Odometry &msg) {
      auto p = msg.pose.pose.position;
      auto q = msg.pose.pose.orientation;
      current_pose[0] = p.x;
      current_pose[1] = p.y;
      auto t3 = 2.0*(q.w * q.z + q.x * q.y);
      auto t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      z_angle = std::atan2(t3, t4);
  }

};  // class ParkingActionServer

}  // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(missions_action_cpp::ParkingActionServer)