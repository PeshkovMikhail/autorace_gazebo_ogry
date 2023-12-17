#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <cv_bridge/cv_bridge.h>   
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "nav_msgs/msg/odometry.hpp"

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
	
}

typedef struct point3
{
	float x,y,z;
	float r,g,b;
}point3;

typedef struct RGB8
{
	unsigned char r,g,b;
}RGB8;

template <typename T>
class vec
{
public:
	T x=0,y=0;
	
	vec operator+(const vec& s) const
	{
		return vec{x+s.x,y+s.y};
	}
	vec operator-(const vec& s) const
	{
		return vec{x-s.x,y-s.y};
	}
	vec operator/(const T value) const
	{
		return vec{x/value,y/value};
	}
	vec operator*(const T value) const
	{
		return vec{x*value,y*value};
	}
	T operator*(const vec& s) const
	{
		return x*s.x+y*s.y;
	}
	vec normalize()
	{
		T l = (T)sqrt(x*x + y*y);
		if (l!=0)
			return vec{x/l,y/l};
		return *this;
	}
	template <typename T1>
	vec(const vec<T1>& s)
	{
		x = (T)s.x;
		y = (T)s.y;
	}
	vec(T x_,T y_)
	{
		x = x_;
		y = y_;
	}
	vec()
	{
		x=0;
		y=0;
	}
	vec round()
	{
		return vec{std::round(x),std::round(y)};
	}
	T len()
	{
		return sqrt(x*x + y*y);
	}
	vec rotate(float angle)
	{
		return vec{x*cos(angle)-y*sin(angle),x*sin(angle)+y*cos(angle)};
	}
	
};

template <typename T>
std::ostream& operator<<(std::ostream& os,const vec<T> vector)
{
	os<<"("<<vector.x<<", "<<vector.y<<")";
	return os;
}	

template <typename T>
class ray
{
public:
	vec<T> pt,dir;
};


float convert_to_gray(RGB8 cl)
{
	return ((float)cl.r+(float)cl.b+(float)cl.g)/(3*255.0f);
}


