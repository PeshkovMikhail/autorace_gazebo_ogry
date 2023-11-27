#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

typedef struct RGB8
{
	unsigned char r,g,b;
}RGB8;

float convert_to_gray(RGB8 cl)
{
	return ((float)cl.r+(float)cl.b+(float)cl.g)/(3*255.0f);
}

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
		
	}

private:
	sensor_msgs::msg::LaserScan lidar;
	float* depth = nullptr;
	float lidar_val = 0;
	int lidar_id = 0;
	float padding=0.5;

	inline float depth_remap(float dep)
	{
		if (dep>100)
			return 100;
		return dep;
	}
	inline bool is_road(const RGB8* image,float* dph,uint32_t idx)
	{
		return (convert_to_gray(image[idx])*depth_remap(dph[idx])<1);
	}
	void save_depth_data(const sensor_msgs::msg::Image& msg)
	{
		if (!depth)
			depth = new float[msg.width*msg.height];
		std::memcpy(depth,msg.data.data(),sizeof(float)*msg.width*msg.height);
	}
	void save_lidar_data(const sensor_msgs::msg::LaserScan& msg)
	{
		lidar = msg;
	}
    void send(const sensor_msgs::msg::Image& msg)
    {
 		if (!depth)
			return;
		geometry_msgs::msg::Twist res;
	
		for (uint32_t i = 1;i<lidar.ranges.size()/4;i++)
			if (lidar.ranges[i]<0.5)
			{
				res.angular.z -= (float)(lidar.ranges.size()/4-i)/(lidar.ranges.size()/4)*0.5;
				// std::cout<<"asdasf"<<std::endl;
			}
				

		for (uint32_t i = lidar.ranges.size()-2;i>lidar.ranges.size()*3/4;i--)
			if (lidar.ranges[i]<0.5)
			{
				res.angular.z += (float)(i-lidar.ranges.size()*3/4)/(lidar.ranges.size()*3/4)*0.5;
				
			}
				

		
        
		const uint32_t strings_cnt = 10;
		const RGB8 *image;
		float *dph;
		rclcpp::Rate loop_rate(10);
		for (uint32_t i = 0;i<strings_cnt;i++)
		{
			image = &(reinterpret_cast<const RGB8*>(msg.data.data())[(msg.height-i*5-1)*(msg.width)]);
			dph = &depth[(msg.height-i*5-1)*(msg.width)];
			uint32_t curr1 = msg.width/2,curr2 = msg.width/2;
			// std::cout<<depth_remap(depth[(msg.height-i*5-1)*(msg.width)])<<std::endl;
			
			while (curr1>=1 && !is_road(image,dph,curr1))
				curr1--;
	
			while (curr2<msg.width-1 && !is_road(image,dph,curr2))
				curr2++;
			// std::cout<<convert_to_gray(image[curr2])<<" "<<curr2<<std::endl;
			
			if (msg.width/2-curr1 < curr2-msg.width/2)
				curr2 = curr1;	
			else
				curr1 = curr2;
			

			while (curr1>=1 && is_road(image,dph,curr1))
					curr1--;
			// std::cout<<convert_to_gray(image[curr1])*depth_remap(dph[0])<<" "<<curr1<<std::endl;
			
			while (curr2<msg.width-1 && is_road(image,dph,curr2))
				curr2++;
			
				
			
			int diff = (int)(curr2+curr1)/2-(int)msg.width/2;
			
			float wtf = (float)(strings_cnt-i)/strings_cnt;
			if (std::abs(diff)<(20*(i+1)*tan(40.0f/180.0f*M_PI)))
				res.linear.x += 0.5*wtf*wtf;
			else
				res.angular.z += -atan2(diff,5*(i+1))*wtf*wtf*wtf;			
			

		}
		// res.linear.x += 0.08;
		if (depth[msg.height/2*msg.width + msg.width/2]>100)
			res.linear.x -= 0.6;
		res.angular.z *= 2.0f/(2+strings_cnt);
		res.linear.x *= 2.0f/(1+strings_cnt);
		
        publisher_->publish(res);
		
		
		// rclcpp::shutdown();
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;

};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalPublisher>());
	rclcpp::shutdown();
	return 0;
}