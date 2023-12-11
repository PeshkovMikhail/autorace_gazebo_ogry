#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

class ImageCapture : public rclcpp::Node {
public:
    ImageCapture() : rclcpp::Node("image_capture") {
        cam_subscription_ = create_subscription<sensor_msgs::msg::Image>("/color/image", 10, std::bind(&ImageCapture::update_image, this, std::placeholders::_1));
        save_subscription_ = create_subscription<std_msgs::msg::String>("/save_img", 10, std::bind(&ImageCapture::save_image, this, std::placeholders::_1));

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr save_subscription_;

    cv::Mat img_mat;

    void save_image(const std_msgs::msg::String& msg) {
        cv::imwrite(msg.data, img_mat);
    }

    void update_image(const sensor_msgs::msg::Image& msg) {
        auto img = cv_bridge::toCvCopy(msg, "bgr8");
        img_mat = img->image.clone();
    }
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageCapture>());
	rclcpp::shutdown();
	return 0;
}