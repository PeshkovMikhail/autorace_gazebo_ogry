#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
// #include 

#include <memory>
#include <utility>


static std::pair<int, int> maskYellowLane(cv::Mat& mat) {
    auto matrix_size = {mat.size[0], mat.size[1]};
    auto hsv = cv::Mat(matrix_size, CV_8UC3);
    auto t = hsv.data;
}

class DriverV2 : public rclcpp::Node {
public:
    DriverV2() : rclcpp::Node("driver_v2") {
        cam_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image", 10, std::bind(&DriverV2::process_image, this, std::placeholders::_1));
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DriverV2::save_lidar_data, this, std::placeholders::_1)
        );

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10
        );


    }
private:
    int hue_white_l = 0;
    int hue_white_h = 179;
    int saturation_white_l = 0;
    int saturation_white_h = 70;
    int lightness_white_l = 105;
    int lightness_white_h = 255;

    int hue_yellow_l = 10;
    int hue_yellow_h = 127;
    int saturation_yellow_l = 70;
    int saturation_yellow_h = 255;
    int lightness_yellow_l = 95;
    int lightness_yellow_h = 255;

    int counter_drive = 0;
    int counter_recognize = 0;

    float window_width = 1000;
    float window_height = 600;
    int reliability_white_line = 100;
    int reliability_yellow_line = 100;


    void save_lidar_data(sensor_msgs::msg::LaserScan& msg) {
        lidar_msg = msg;
    }

    void process_image(const sensor_msgs::msg::Image& msg) {
        auto mat = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    void find_line(cv::Mat& mat) {
        counter_drive++;
        if (counter_drive % 3 != 0) {
            return;
        }


    }


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;


    sensor_msgs::msg::LaserScan lidar_msg;


    

};


int main() {

}