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
#include "geometry_msgs/msg/twist.hpp"

using namespace std::placeholders;

float calcMSE(float arr1[2], float arr2[2]) {
  float x = arr1[0] - arr2[0];
  x *= x;
  float y = arr1[1] - arr2[1];
  y *= y;
  return (x+y)/2;
}


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
        goal_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        finished_ = create_publisher<std_msgs::msg::String>("/robot_finished", 10);
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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr finished_;
    rclcpp_action::Server<Tunnel>::SharedPtr action_server_;

    float* depth = nullptr;
    sensor_msgs::msg::LaserScan lidar;
    float z_angle = NAN;
    bool activated = false;

    float current_pose[2] = {0, 0};
    float finish_pose[2] = {0, 0};

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
        auto p = msg.pose.pose.position;
        auto q = msg.pose.pose.orientation;
        current_pose[0] = p.x;
        current_pose[1] = p.y;
        auto t3 = 2.0*(q.w * q.z + q.x * q.y);
        auto t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        z_angle = std::atan2(t3, t4);
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

        std_msgs::msg::Bool enabled;
        enabled.data = false;

        geometry_msgs::msg::Twist twist;
    

        twist.linear.x = 0.2;
        vel_publisher_->publish(twist);

        driver_state_pub_->publish(enabled);

        // float start = get_clock()->now().seconds();

        // while (get_clock()->now().seconds() - start < 3)
        // {
        //     loop_rate.sleep();
        // }


        

        twist.linear.x = 0.4;
        vel_publisher_->publish(twist);
        while(lidar.ranges[0]>0.5)
        {
            loop_rate.sleep();
        }
        twist.linear.x = 0.1;
        vel_publisher_->publish(twist);
        while(lidar.ranges[0]>0.2)
        {
            loop_rate.sleep();
        }
        twist.linear.x = 0;
        twist.angular.z = M_PI/2;

        vel_publisher_->publish(twist);
        auto angle = z_angle + M_PI/2;
        while(abs(z_angle-angle)>0.01)
        {
            loop_rate.sleep();
        }
        twist.linear.x = 1;
        vel_publisher_->publish(twist);

        driver_state_pub_->publish(enabled);

        float start = get_clock()->now().seconds();

        while (get_clock()->now().seconds() - start < 3)
        {
            loop_rate.sleep();
        }
        

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "robot/map";
        pose.header.stamp = get_clock()->now();

        
        
        goal_pose_->publish(pose);

        // get_clock()->now();


        while(calcMSE(current_pose, finish_pose) > 0.05) {
            loop_rate.sleep();
        }

        std_msgs::msg::String msg;
        msg.data = "gazebo_ogry";
        finished_->publish(msg);

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