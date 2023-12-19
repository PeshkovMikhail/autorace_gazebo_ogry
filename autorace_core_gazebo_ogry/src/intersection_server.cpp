#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "autorace_communication_gazebo_ogry/action/intersection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "autorace_core_gazebo_ogry/intersection_visibility.h"

enum class Direction {
  None,
  Left,
  Right
};

float calcMSE(float arr1[2], float arr2[2]) {
  float x = arr1[0] - arr2[0];
  x *= x;
  float y = arr1[1] - arr2[1];
  y *= y;
  return (x+y)/2;
}

double calculateRotationAngle(double startAngle, double endAngle) {
    // Разница между конечным и начальным углом
    double angleDifference = endAngle - startAngle;

    // Приведение разницы к диапазону (-pi, pi]
    angleDifference = fmod((angleDifference + M_PI), (2 * M_PI)) - M_PI;

    return angleDifference;
}
    

namespace missions_action_cpp
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
    turn_dir_subscription_= create_subscription<std_msgs::msg::String>("/intersection/turn_dir", 10, std::bind(&IntersectionActionServer::set_turn_dir, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&IntersectionActionServer::update_odom, this, std::placeholders::_1));

    vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    driver_state_ = create_publisher<std_msgs::msg::Bool>("/driver_state", 10);
    driver_line_prop_ = create_publisher<std_msgs::msg::Float32>("/change_cringe", 10);
    is_consturct = create_publisher<std_msgs::msg::Bool>("/is_construct", 10);
    
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr turn_dir_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr driver_state_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr driver_line_prop_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_consturct;

  float current_pose[2] = {0, 0};
  float stop_pose[2] = {0.70f, 0.98f};
  float turn_pose[2] = {0.47f, 0.98f};
  float finish_pose[2] = {-0.35, 0.90};
  float z_angle;
  bool await_turn_dir = false;

  Direction dir = Direction::None;

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
    // // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&IntersectionActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleIntersection> goal_handle)
  {
    rclcpp::Rate loop_rate(50);
    await_turn_dir = true;
    RCLCPP_INFO(get_logger(), "INTERSECTION TASK STARTED");
    
    // while(calcMSE(current_pose, stop_pose) > 0.001) {
    //   loop_rate.sleep();
    //   //RCLCPP_INFO(get_logger(), "%f", calcMSE(current_pose, goal_pose));
    // }
    // auto driver_state_msg = std_msgs::msg::Bool();
    // auto twist = geometry_msgs::msg::Twist();
   
    
    // driver_state_msg.data = false;
    // driver_state_->publish(driver_state_msg);
    // twist.linear.x = 0;
    // vel_publisher_->publish(twist);

    // if(dir == Direction::None){
    //   dir = Direction::Left;
    // }
    // await_turn_dir = false;
    // twist.linear.x = 0.1;
    // vel_publisher_->publish(twist);

    // while(calcMSE(current_pose, turn_pose) > 0.01) {
    //   loop_rate.sleep();
    //   //RCLCPP_INFO(get_logger(), "%f", calcMSE(current_pose, goal_pose));
    // }

    // switch (dir)
    // {
    // case Direction::Right:
    //   turn_to_angle(3*3.14/4, loop_rate, -1);
    //   break;
    // case Direction::Left:
    //   turn_to_angle(-3*3.14/4, loop_rate, 1);
    //   break;
    // case Direction::None:
    //   RCLCPP_ERROR(get_logger(), "failed to detect turn direction");
    //   break;
    // default:
    //   break;
    // }
    // twist.linear.x = 0.05;
    // vel_publisher_->publish(twist);
    // driver_state_msg.data = true;
    // driver_state_->publish(driver_state_msg);

    // Check if goal is done
    while(calcMSE(current_pose, finish_pose) > 0.015) {
      loop_rate.sleep();
      //RCLCPP_INFO(get_logger(), "%f", calcMSE(current_pose, goal_pose));
    }
    await_turn_dir = false;
    std_msgs::msg::Float32 prop;
    prop.data = 0.5;
    driver_line_prop_->publish(prop);

    sleep(10);
    std_msgs::msg::Bool aezakmi;
    aezakmi.data = true;
    is_consturct->publish(aezakmi);
    // driver_state_msg.data = false;
    // driver_state_->publish(driver_state_msg);

    // int sign = dir == Direction::Right ? -1 : 1;
    // turn_to_angle(sign*5*3.14/6, loop_rate, sign);

    // driver_state_msg.data = true;
    // driver_state_->publish(driver_state_msg);
    
    auto result = std::make_shared<Intersection::Result>();
    if (rclcpp::ok()) {
      result->finished = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "INTERSECTION TASK FINISHED");
    }
  }


  void update_lidar(const sensor_msgs::msg::LaserScan &msg) {
    // msg.ranges
  }

  void turn_to_angle(float angle, rclcpp::Rate &loop_rate, int sign) {
    auto twist = geometry_msgs::msg::Twist();
    twist.angular.z = sign*0.5;

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

  void set_turn_dir(const std_msgs::msg::String &msg) {
    if(dir != Direction::None) {
      return;
    }
    std_msgs::msg::Float32 prop;
    prop.data = 0.5;
    if(!await_turn_dir) {
      return;
    }
    if(msg.data == "left") {
      dir = Direction::Left;
      prop.data = 0.2;
    }
    else if(msg.data == "right") {
      dir = Direction::Right;
      prop.data  = 0.8;
    }
    else{
      RCLCPP_ERROR(get_logger(), "unexpected direction");
    }
    driver_line_prop_->publish(prop);
    RCLCPP_INFO(get_logger(), "turn %s", msg.data.c_str());
  }
};  // class IntersectionActionServer

}  // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(missions_action_cpp::IntersectionActionServer)