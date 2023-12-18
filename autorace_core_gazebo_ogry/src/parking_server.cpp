#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "autorace_communication_gazebo_ogry/action/parking.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "autorace_core_gazebo_ogry/parking_visibility.h"

float calcMSE(float arr1[2], float arr2[2]) {
  float x = arr1[0] - arr2[0];
  x *= x;
  float y = arr1[1] - arr2[1];
  y *= y;
  return (x+y)/2;
}

enum class Direction {
  None,
  Left,
  Right
};

float turn_pose[2] = {-0.6, 3.34};
float stop_pose[2] = {-0.6, 2.8};

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

    lidar_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, std::bind(&ParkingActionServer::update_lidar, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&ParkingActionServer::update_odom, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>("/depth/image", 1, std::bind(&ParkingActionServer::update_depth, this, std::placeholders::_1));

    vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    driver_state_ = create_publisher<std_msgs::msg::Bool>("/driver_state", 10);
    driver_vel_ = create_publisher<std_msgs::msg::Float32>("/driver_vel", 10);
    
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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr driver_state_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr driver_vel_;

  float current_pose[2] = {0, 0};
  float goal_pose[2] = {-0.55f, 3.51f};
  float finish_pose[2] = {-0.11, 0.98};
  float z_angle;

  float* depth = nullptr;

  sensor_msgs::msg::LaserScan lidar;

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
    std_msgs::msg::Bool driver_state;
    std_msgs::msg::Float32 drive_vel;
    geometry_msgs::msg::Twist twist;

    drive_vel.data = 0.2;
    driver_vel_->publish(drive_vel);
    
    RCLCPP_INFO(get_logger(), "PARKING TASK STARTED");

    while(calcMSE(current_pose, turn_pose) > 0.01) {
      loop_rate.sleep();
    }
    driver_state.data = false;
    driver_state_->publish(driver_state);

    turn_to_angle(-3.14/2, loop_rate, 1);

    driver_state.data = true;
    driver_state_->publish(driver_state);

    while(calcMSE(current_pose, stop_pose) > 0.002) {
      loop_rate.sleep();
    }
    driver_state.data = false;
    driver_state_->publish(driver_state);

    int lidar_size = (lidar.angle_max - lidar.angle_min)/lidar.angle_increment;
    int i;
    bool found_car = false;
    for(i = 0; i < lidar_size; i++) {
      if(lidar.ranges[i] > lidar.range_min &&  lidar.ranges[i] < 0.5) {
        found_car = true;
        break;
      }
    }
    float angle = lidar.angle_min + lidar.angle_increment*i;

    auto dir = Direction::None;
    if(found_car) {
      if(angle > 0) {
        dir = Direction::Left;
      }
      else {
        dir = Direction::Right;
      }
    }
    else {
      RCLCPP_INFO(get_logger(), "No parked car found. Using left place");
      dir = Direction::Left;
    }
    
    if(dir == Direction::Left) {
      turn_to_angle(0, loop_rate, 1);
    }
    else {
      turn_to_angle(3.14, loop_rate, -1);
    }

    twist.linear.x = 0.2;
    vel_publisher_->publish(twist);

    while(!std::isinf(depth[848*450+ 424])) {
            loop_rate.sleep();
    }

    if(dir == Direction::Left) {
      turn_to_angle(3.14/2, loop_rate, 1);
    }
    else {
      turn_to_angle(3.14/2, loop_rate, -1);
    }

    twist.linear.x = 0.2;
    vel_publisher_->publish(twist);

    while(!std::isinf(depth[848*450+ 424])) {
            loop_rate.sleep();
    }

    if(dir == Direction::Left) {
      turn_to_angle(3.14, loop_rate, 1);
    }
    else {
      turn_to_angle(0, loop_rate, -1);
    }

    twist.linear.x = 0.1;
    vel_publisher_->publish(twist);

    while(lidar.ranges[0] > 0.25) {
      loop_rate.sleep();
    }
    if(dir == Direction::Right) {
      turn_to_angle(3.14/2, loop_rate, 1);
    }
    else{
      turn_to_angle(3.14/2, loop_rate, -1);

    }
    
    twist.linear.x = 0.2;
    vel_publisher_->publish(twist);

     while(!std::isinf(depth[848*400+ 424])) {
            loop_rate.sleep();
    }

    turn_to_angle(3.14, loop_rate, 1);

    drive_vel.data = 0.3;
    driver_vel_->publish(drive_vel);

    driver_state.data = true;
    driver_state_->publish(driver_state);
    
    auto result = std::make_shared<Parking::Result>();
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "PARKING TASK FINISHED");
    }
  }


  void update_lidar(const sensor_msgs::msg::LaserScan &msg) {
    lidar = msg;
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

  void update_depth(const sensor_msgs::msg::Image& msg) {
        if(depth == nullptr) {
            depth = new float[msg.width*msg.height];
        }
        std::memcpy(depth, msg.data.data(), sizeof(float)*msg.height*msg.width);
    }

};  // class ParkingActionServer

}  // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(missions_action_cpp::ParkingActionServer)