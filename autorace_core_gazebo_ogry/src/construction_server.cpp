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
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&ConstructionServer::update_odom, this, std::placeholders::_1));
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
    rclcpp_action::Server<Obstacles>::SharedPtr action_server_;

    float* depth = nullptr;
    sensor_msgs::msg::LaserScan lidar;
    float z_angle = NAN;
    bool activated = false;

    void update_depth(const sensor_msgs::msg::Image& msg) {
        if(depth == nullptr) {
            depth = new float[msg.width*msg.height];
        }
        std::memcpy(depth, msg.data.data(), sizeof(float)*msg.height*msg.width);
    }

    void update_lidar(const sensor_msgs::msg::LaserScan& msg) {
        lidar = msg;
    }

    void update_odom(const nav_msgs::msg::Odometry& msg) {

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
        

        auto result = std::make_shared<Obstacles::Result>();
        if (rclcpp::ok()) {
            // result->finished = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "OBSTACLES TASK FINISHED");
        }
    }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(missions_action_cpp::ConstructionServer)