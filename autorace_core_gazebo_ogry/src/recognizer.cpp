#include <chrono>
#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/opencv.hpp"


class Recognizer : public rclcpp::Node {
public:
	Recognizer() : rclcpp::Node("recognizer") {
		std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_bringup");
		std::string intersection = package_share_directory + "/worlds/intersection/";
		std::string materials = package_share_directory + "/worlds/materials/";
	
		traffic_left = read_image(intersection + "traffic_left.png");
		traffic_right = read_image(intersection + "traffic_right.png");

		pedestrian = read_image(materials + "pedestrian_crossing_sign.png");
		traffic_construction = read_image(materials + "traffic_construction.png");
		traffic_intersection = read_image(materials + "traffic_intersection.png");
		traffic_parking = read_image(materials + "traffic_parking.png");
		tunnel = read_image(materials + "tunnel.png");

		cam_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image", 10, std::bind(&Recognizer::recognize, this, std::placeholders::_1));
	}
private:
	cv::Mat traffic_left, traffic_right;
	cv::Mat pedestrian, traffic_construction, traffic_intersection, traffic_parking, tunnel;

	cv::Mat read_image(std::string path) {
		auto res = cv::imread(path);
		if(!res.data) {
			RCLCPP_ERROR(get_logger(), "reading %s failed", path.c_str());
			rclcpp::shutdown();
		}
		return res;
	}

	void recognize(const sensor_msgs::msg::Image& msg) {
		// cv::Mat img = cv::Mat(1, msg.data, )
	}

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_subscription_;
};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Recognizer>());
	rclcpp::shutdown();
	return 0;
}