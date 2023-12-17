#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "autorace_communication_gazebo_ogry/action/tunnel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "autorace_core_gazebo_ogry/tunnel_visibility.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::placeholders;


namespace missions_action_cpp
{
class TunnelServer : public rclcpp::Node {
public:
using Tunnel = autorace_communication_gazebo_ogry::action::Tunnel;
using GoalHandleTunnel = rclcpp_action::ServerGoalHandle<Tunnel>;
    TUNNEL_ACTION_CPP_PUBLIC
    explicit TunnelServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("Tunnel_action_server") {
        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, std::bind(&TunnelServer::update_lidar, this, std::placeholders::_1));
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>("/depth/image", 1, std::bind(&TunnelServer::update_depth, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&TunnelServer::update_odom, this, std::placeholders::_1));

        driver_state_pub_ = create_publisher<std_msgs::msg::Bool>("/driver_state", 10);
        last_task_pub_ = create_publisher<std_msgs::msg::Bool>("/last_task", 10);
        this->action_server_ = rclcpp_action::create_server<Tunnel>(
            this,
            "tunnel",
            std::bind(&TunnelServer::handle_goal, this, _1, _2),
            std::bind(&TunnelServer::handle_cancel, this, _1),
            std::bind(&TunnelServer::handle_accepted, this, _1)
        );
        
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr driver_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr last_task_pub_;
    rclcpp_action::Server<Tunnel>::SharedPtr action_server_;

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
    std::shared_ptr<const Tunnel::Goal> goal)
    {
        // RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTunnel> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleTunnel> goal_handle)
    {
        using namespace std::placeholders;
        // // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&TunnelServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleTunnel> goal_handle)
    {
        rclcpp::Rate loop_rate(50);
        std_msgs::msg::Bool msg;
        msg.data = true;
        last_task_pub_->publish(msg);
        while (true)
        {
            loop_rate.sleep();
        }
        
        // std_msgs::msg::Bool enabled;
        // enabled.data = false;

        // driver_state_pub_->publish(enabled);

        // geometry_msgs::msg::PoseStamped pose;
        // pose.header.frame_id = "map";
        // pose.header.stamp = get_clock()->now();
        


        // get_clock()->now();


        


        auto result = std::make_shared<Tunnel::Result>();
        if (rclcpp::ok()) {
            // result->finished = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Tunnel TASK FINISHED");
        }
    }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(missions_action_cpp::TunnelServer)