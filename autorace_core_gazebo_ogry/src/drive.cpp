#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "road.hpp"


    




using namespace std::chrono_literals;






class MinimalPublisher : public rclcpp::Node
{
public:
	MinimalPublisher()
	: Node("minimal_publisher")
	{
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		cam_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image", 10, std::bind(&MinimalPublisher::send, this, std::placeholders::_1));
		depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth/image", 10, std::bind(&MinimalPublisher::save_depth_data, this, std::placeholders::_1));
		lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MinimalPublisher::save_lidar_data, this, std::placeholders::_1));
		// pointCloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/depth/points", 10, std::bind(&MinimalPublisher::save_pointCloud_data, this, std::placeholders::_1));
		show_pub = this->create_publisher<sensor_msgs::msg::Image>("/show_line", 10);
		driver_state_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
			"/driver_state", 10, std::bind(&MinimalPublisher::set_enabled, this, std::placeholders::_1));
		
		
	}

private:
	const uint32_t width=848;
	const uint32_t height=480;
	sensor_msgs::msg::LaserScan lidar;
	IIS iis;
	float* depth = nullptr;
	point3* cloud = nullptr;
	float lidar_val = 0;
	int lidar_id = 0;
	float padding=0.5;
	vec<float> start_vec = {0,1};

	float last_lin = 0;
	float last_ang = 0;
	
	float k_dif=0.2;

	bool driver_state_ = false;

	
	void save_depth_data(const sensor_msgs::msg::Image& msg)
	{
		if (!depth)
			depth = new float[width*height];
		std::memcpy(depth,msg.data.data(),sizeof(float)*msg.width*msg.height);
	}
	void save_lidar_data(const sensor_msgs::msg::LaserScan& msg)
	{
		lidar = msg;
	}
	
	
	
    void send(const sensor_msgs::msg::Image& msg)
    {
		//PROJECT RENAV
		//---------`_\ 
		//     <-     |
		//------\  ^   |
		//      |  |   |
		////////////////
		if (!depth || !driver_state_)
			return;

		auto img_orig = cv_bridge::toCvCopy(msg, "bgr8")->image;
		cv_bridge::CvImagePtr show= cv_bridge::toCvCopy(msg, "rgb8");
		cv::Mat img;
		cv::cvtColor(img_orig, img, cv::COLOR_BGR2HSV);
		// cv::cvtColor(cv_bridge::toCvCopy(msg, "bgr8")->image);
		// show->image.at<RGB8>(479 - i, (int)pt) = {255, 0, 255};

		// auto image = (RGB8*)msg.data.data();
		iis.depth = depth;
		iis.image = img;
		//show->image.ptr<RGB8>(479-1)[(int)2]= {255,0,255};
		

		iis.show = show;
		auto vector = iis.mrv();
		
		geometry_msgs::msg::Twist res;

		res.linear.x = vector.y*0.2;
		res.angular.z = -atan2(vector.x,vector.y)*0.7;
		

		res.angular.z = (1-k_dif)*res.angular.z + k_dif*last_ang;
		res.linear.x = (1-k_dif)*res.linear.x + k_dif*last_lin;
		if (std::isnan(res.angular.z) || std::isnan(res.linear.x))
		{
			res.angular.z = 0;
			res.angular.x = 0;
			RCLCPP_INFO(this->get_logger(),"HUETA: %f\n",res.linear.x);
		}
		else
		{
			publisher_->publish(res);
			cv_bridge::CvImage out_msg;
			out_msg.header = msg.header;
			out_msg.encoding = "rgb8";
			out_msg.image = img_orig;
			show_pub->publish(*show->toImageMsg());
		}
        	
		
		last_ang = res.angular.z;
		last_lin = res.linear.x;
		// RCLCPP_INFO(get_logger(), "%f %f", last_lin, last_ang);
		// rclcpp::shutdown();
    }

	void set_enabled(const std_msgs::msg::Bool& msg) {
		driver_state_ = msg.data;
	}

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_subscription_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr show_pub;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driver_state_subscription_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalPublisher>());
	rclcpp::shutdown();
	return 0;
}