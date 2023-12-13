#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"


#define THRESHOLD 150

struct Sign {
    std::string name;
    std::vector<cv::KeyPoint> kp;
    cv::Mat des;
};

class SignDetector : public rclcpp::Node {
public:
    SignDetector() : Node("sign_detector") {
        std::string path = ament_index_cpp::get_package_share_directory("autorace_vision_gazebo_ogry") + "/images/";
        orb = cv::ORB::create();
        matcher = cv::BFMatcher(cv::NORM_HAMMING);

        // Detect keypoints and compute descriptors
        

        std::vector<std::string> names = {"intersection", "construction", "parking", "crosswalk", "tunnel"};

        for(auto name: names) {
            auto img = read_image(path + name + ".png");
            Sign sign = {name, std::vector<cv::KeyPoint>(), cv::Mat()};
            orb->detectAndCompute(img, cv::noArray(), sign.kp, sign.des);
            signs.push_back(sign);
        }

        camera_sub_ = create_subscription<sensor_msgs::msg::Image>("/color/image", 1, std::bind(&SignDetector::detect, this, std::placeholders::_1));
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;

    std::vector<Sign> signs;
    cv::Ptr<cv::Feature2D> orb;
    cv::BFMatcher matcher;

    void detect(const sensor_msgs::msg::Image& msg) {
        auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");
        auto crop = cv_image->image;
        cv::Mat grayscale;
        cv::cvtColor(crop, grayscale, cv::COLOR_BGR2GRAY);

        std::vector<cv::KeyPoint> kp;
        cv::Mat des;
        orb->detectAndCompute(grayscale, cv::noArray(), kp, des);
        int max_kp = 0;
        std::string name = "nope";
        for(auto sign : signs) {
            std::vector<cv::DMatch> matches, good_matches;
            matcher.match(des, sign.des, matches);
            double min_dist = 100;
            for( int i = 0; i < matches.size(); i++ )
            { 
                double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
            }
            for(int i = 0; i < matches.size(); i++ )
            { 
                if( matches[i].distance < 1.5*min_dist )
                { good_matches.push_back( matches[i]); }
            }
            if(good_matches.size() >= THRESHOLD && good_matches.size() > max_kp) {
                max_kp = good_matches.size();
                name = sign.name;
            }
        }
        if(max_kp >= THRESHOLD) {
            RCLCPP_INFO(get_logger(), "%s %d", name.c_str(), max_kp);
        }
    }


    cv::Mat read_image(std::string path) {
		auto res = cv::imread(path, cv::IMREAD_GRAYSCALE);
		if(!res.data) {
			RCLCPP_ERROR(get_logger(), "reading %s failed", path.c_str());
			rclcpp::shutdown();
		}
		
		return res;
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SignDetector>());
	rclcpp::shutdown();
	return 0;
}