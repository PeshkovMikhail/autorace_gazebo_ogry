#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "autorace_communication_gazebo_ogry/action/obstacles.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "autorace_core_gazebo_ogry/construction_visibility.h"

using namespace std::placeholders;



namespace missions_action_cpp
{
class ConstructionServer : public rclcpp::Node {
public:
using Obstacles = autorace_communication_gazebo_ogry::action::Obstacles;
using GoalHandleCrosswalk = rclcpp_action::ServerGoalHandle<Obstacles>;
    CONSTRUCTION_ACTION_CPP_PUBLIC
    explicit ConstructionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("construction_action_server") {
        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, std::bind(&ConstructionServer::update_lidar, this, std::placeholders::_1));
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>("/depth/image", 1, std::bind(&ConstructionServer::update_depth, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&ConstructionServer::update_odom, this, std::placeholders::_1));

        vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        driver_state_pub_ = create_publisher<std_msgs::msg::Bool>("/driver_state", 10);

        this->action_server_ = rclcpp_action::create_server<Obstacles>(
            this,
            "construction",
            std::bind(&ConstructionServer::handle_goal, this, _1, _2),
            std::bind(&ConstructionServer::handle_cancel, this, _1),
            std::bind(&ConstructionServer::handle_accepted, this, _1)
        );
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr driver_state_pub_;
    rclcpp_action::Server<Obstacles>::SharedPtr action_server_;

    float* depth = nullptr;
    sensor_msgs::msg::LaserScan lidar;
    float z_angle = NAN;
    bool activated = false;

    float current_pose[2];

    void update_depth(const sensor_msgs::msg::Image& msg) {
        if(depth == nullptr) {
            depth = new float[msg.width*msg.height];
        }
        std::memcpy(depth, msg.data.data(), sizeof(float)*msg.height*msg.width);
    }

    void update_lidar(const sensor_msgs::msg::LaserScan& msg) {
        lidar = msg;
    }


    rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Obstacles::Goal> goal)
    {
        // RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCrosswalk> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCrosswalk> goal_handle)
    {
        using namespace std::placeholders;
        // // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ConstructionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleCrosswalk> goal_handle)
    {
        rclcpp::Rate loop_rate(50);
        geometry_msgs::msg::Twist twist;
        std_msgs::msg::Bool driver_state;
        

        while(lidar.ranges.at(0) > 0.3) {
            loop_rate.sleep();
        }
        driver_state.data = false;
        driver_state_pub_->publish(driver_state);

        twist.linear.x = 0;
        vel_publisher_->publish(twist);

        turn_to_angle(3.14, loop_rate, 1);

        twist.linear.x = 0.1;
        vel_publisher_->publish(twist);

        while(!std::isinf(depth[848*450+ 424])) {
            loop_rate.sleep();
        }

        turn_to_angle(3.14/2, loop_rate, -1);

        twist.linear.x = 0.2;
        vel_publisher_->publish(twist);

        while(lidar.ranges.at(0) > 0.3) {
            loop_rate.sleep();
        }

        turn_to_angle(0, loop_rate, -1);

        twist.linear.x = 0.1;
        vel_publisher_->publish(twist);

        while(!std::isinf(depth[848*450+ 424])) {
            loop_rate.sleep();
        }

        turn_to_angle(3.14/2, loop_rate, 1);

        twist.linear.x = 0.1;
        vel_publisher_->publish(twist);

        while(!std::isinf(depth[848*420+ 424])) {
            loop_rate.sleep();
        }

        driver_state.data = true;
        driver_state_pub_->publish(driver_state);
        

        auto result = std::make_shared<Obstacles::Result>();
        if (rclcpp::ok()) {
            // result->finished = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "OBSTACLES TASK FINISHED");
        }
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

};
}

RCLCPP_COMPONENTS_REGISTER_NODE(missions_action_cpp::ConstructionServer)