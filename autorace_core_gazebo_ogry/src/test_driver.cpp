#include <chrono>
#include <memory>
#include <cmath>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "cv_bridge/cv_bridge.h"
#include "pcl/common/eigen.h"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

#define CAM_ANGLE 0.11
#define CAM_FOV_HOR 1.51843645
#define CAM_HF 480.0
#define CAM_H 480
#define CAM_WF 848.0
#define CAM_W 848
#define CAM_FOV_VER (2.0*std::atan(std::tan(CAM_FOV_HOR/2.0)*CAM_HF/CAM_WF))

#define ITER_COUNT 10
#define VEC_SIZE 3
#define MAX_ANGLE (10.0*3.1415/180.0)
#define VEC_FADING 0.5

using Eigen::Vector2i;
using Eigen::Vector2f;

enum class Orientation {
    Horizontal,
    Vertical
};

typedef struct {
    float a;
    float b;
    float c;
    float d;
} Params;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

cv::Mat create_mask(cv::Mat img) {
    cv::Mat wmask, ymask;
    cv::inRange(img, cv::Scalar(0, 0, 230), cv::Scalar(179, 70, 255), wmask);
    cv::inRange(img, cv::Scalar(10, 100, 180), cv::Scalar(40, 255, 255), ymask);
    return (wmask | ymask);
}

std::pair<Orientation, Params> build_perpendicular_line(Vector2f v, Vector2f p) {
    Vector2f nv(v.y(), -v.x());
    if (nv.y() != 0 && std::fabs(nv.x()/nv.y()) < 1) {
        Params param = {nv.x(), p.y(), nv.y(), p.x()};
        return std::make_pair(Orientation::Vertical, param);
    }
    Params param = {nv.y(), p.x(), nv.x(), p.y()};
    return std::make_pair(Orientation::Horizontal, param);
}

std::pair<Orientation, std::vector<float>> get_line(Vector2f f_p, Vector2f s_p) {
    std::vector<float> res;
    if(std::fabs(f_p.x() - s_p.x()) < 0.01) {
        res.push_back((f_p.x() - s_p.x())/2.0f);
        return std::make_pair(Orientation::Vertical, res);
    }
    float k = (f_p.y() - s_p.y())/(f_p.x() - s_p.x());
    res.push_back(k); // k
    res.push_back((f_p.y() - k*f_p.x()));
    return std::make_pair(Orientation::Horizontal, res);
}

Vector2f road_coords_2_img(Vector2f coords) {
    float x = coords.x(), y = coords.y();
    // std::cout << "road coords " << x << " " << y << std::endl;
    std::fflush(stdout);
    float i = (x/(y*std::tan(CAM_FOV_HOR/2.0f))+1)/2.0f*(CAM_WF-1.0f);
    float j = (CAM_HF-1.0f)/2.0f*(std::tan(std::atan2(0.10f, y) - CAM_ANGLE)/std::tan(CAM_FOV_VER/2.0f)+1.0f);
    j = CAM_HF - j - 1;
    // std::cout << i << " " << j << std::endl;
    std::fflush(stdout);
    return Vector2f(
        i,
        j
    );
}

std::pair<Orientation, Params> draw_line(Vector2f nv, Vector2f p) {
    if (nv.y() != 0 && std::fabs(nv.x()/nv.y()) < 1) {
        Params param = {nv.x(), p.y(), nv.y(), p.x()};
        return std::make_pair(Orientation::Vertical, param);
    }
    Params param = {nv.y(), p.x(), nv.x(), p.y()};
    return std::make_pair(Orientation::Horizontal, param);
}


float param_func(Params p, float t) {
    return p.a*(t-p.b)/(p.c)+p.d;
    
}

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("driver") {
        cam_sub_ = create_subscription<sensor_msgs::msg::Image>("/color/image", 10, std::bind(&MinimalPublisher::get_image, this, std::placeholders::_1));
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>("/depth/image", 10, std::bind(&MinimalPublisher::get_depth, this, std::placeholders::_1));
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        output_pub_ = create_publisher<sensor_msgs::msg::Image>("/output", 10);
        
        Eigen::Vector2f vec(3, 5);
        
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_pub_;

    float* depth = nullptr;
    cv::Mat mask;
    float last_rotate = 0;


    void get_image(const sensor_msgs::msg::Image& msg) {
        if(!depth) {return;}

        Vector2f PIZDEC;

        auto t = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat hsv_img;
        cv::cvtColor(t, hsv_img, cv::COLOR_BGR2HSV);

        mask = create_mask(hsv_img);
        cv::imwrite("mask.png", mask);

        std::vector<Vector2f> points;
        points.push_back(get_first_pixel());

        std::vector<Vector2f> vectors;
        vectors.push_back(Vector2f(0, VEC_SIZE));
        cv::Mat output = cv_bridge::toCvCopy(msg, "bgr8")->image;

        try {
            auto perp = build_perpendicular_line(vectors.back(), points.back() + vectors.back());
            auto orientation = perp.first;
            Params params = perp.second;

            for(int iter = 0; iter < ITER_COUNT; iter++) {
                // RCLCPP_INFO(get_logger(), "%d iter", iter);
                Vector2f gp = points.back() + vectors.back();
                Vector2f pp = points.back();
                Vector2f vec = vectors.back();

                int iyx, iyy, iwx, iwy;
                if(orientation == Orientation::Horizontal) {
                    iwy = gp.y();
                    for(iwx = std::round(gp.x()); iwx < 848; iwx++) {
                        int pt = param_func(params, iwx);
                        if(!(0 <= pt && pt < 480) || !isroad(Vector2i(iwx, pt))) {
                            iwx -= 1;
                            break;
                        }
                        iwy = pt;
                        output.at<cv::Vec3b>(479-pt, iwx) = cv::Vec3b(255, 0, 255);
                    }

                    iyy = gp.y();
                    for(iyx = std::round(gp.x()); iyx > -1; iyx--) {
                        int pt = param_func(params, iyx);
                        // // RCLCPP_INFO(get_logger(), "%d %d %d", iyx, pt, isroad(Vector2i(iyx, pt)));
                        if(!(0 <= pt && pt < 480) || !isroad(Vector2i(iyx, pt))) {
                            iyx += 1;
                            break;
                        }
                        iyy = pt;
                        output.at<cv::Vec3b>(479-pt, iyx) = cv::Vec3b(255, 0, 255);
                    }
                }
                else {
                    iwx = gp.x();
                    for(iwy = std::round(gp.y()); iwy < 480; iwy++) {
                        int pt = param_func(params, iwy);
                        if(!(0 <= pt && pt < 848) || !isroad(Vector2i(pt, iwy))) {
                            iwy -= 1;
                            break;
                        }
                        iwx = pt;
                        output.at<cv::Vec3b>(479-iwy, iwx) = cv::Vec3b(255, 0, 255);
                    }

                    iyx = gp.x();
                    for(iyy = std::round(gp.y()); iyy > -1; iyy--) {
                        int pt = param_func(params, iyy);
                        if(!(0 <= pt && pt < 848) || !isroad(Vector2i(pt, iyy))) {
                            iyy += 1;
                            break;
                        }
                        iyx = pt;
                        output.at<cv::Vec3b>(479-iyy, iyx) = cv::Vec3b(255, 0, 255);
                    }
                }
                // RCLCPP_INFO(get_logger(), "found boundes");
                // rclcpp::shutdown();

                Vector2f y(iyx, iyy);
                Vector2f w(iwx, iwy);

                // // std::cout << "y " << y << " w " << w<<  std::endl;
                // // RCLCPP_INFO(get_logger(), "y %f %f w %f %f", y.x(), y.y(), w.x(), w.y());

                auto t = check_down(y, w);
                y = t.first;
                w = t.second;
                // std::cout << y << w << std::endl;
                auto p_new = road_coords_2_img((y + w)/2);
                Vector2f n_w = p_new - pp;
                // std::cout << p_new << std::endl << pp << std::endl;
                std::fflush(stdout);
                n_w /= n_w.norm();
                n_w *= VEC_SIZE;
                n_w = n_w*VEC_FADING + vec*(1-VEC_FADING);
                

                // float angle = std::acos(n_w.dot(vec)/(n_w.norm()*vec.norm()));
                // if (std::fabs(angle) > MAX_ANGLE) {
                //     n_w = sgn(angle)*Vector2f(vec.x()*std::cos(MAX_ANGLE)-vec.y()*std::sin(MAX_ANGLE), vec.x()*std::sin(MAX_ANGLE)+vec.y()*std::cos(MAX_ANGLE));
                // }  
                points.push_back(n_w+pp);
                vectors.push_back(n_w);
        
                
                cv::rectangle(output, cv::Point(points.back().x()-2, 479 - points.back().y()+2), cv::Point(points.back().x()+2, 479 - points.back().y()-2), cv::Vec3b(255, 0, 0), cv::FILLED);
                
                Vector2f perp_vec = (y+w).cast<float>()/2.0f-img_coords_translate_2_road(points.at(points.size()-2).cast<int>());
                // RCLCPP_INFO(get_logger(), "vectors check %f %f", perp_vec.x(), perp_vec.y());
                perp_vec = Vector2f(perp_vec.y(), -perp_vec.x());
                perp_vec += img_coords_translate_2_road(points.back().cast<int>());
                // RCLCPP_INFO(get_logger(), "vectors check");
                perp_vec = road_coords_2_img(perp_vec)-points.back();
                // RCLCPP_INFO(get_logger(), "svlolota");
                perp = draw_line(perp_vec, points.back() + vectors.back());
                // RCLCPP_INFO(get_logger(), "final");
            }
            PIZDEC = (img_coords_translate_2_road(points.back().cast<int>()) - img_coords_translate_2_road(points.at(0).cast<int>()));
            PIZDEC /= PIZDEC.norm();
            if(std::isnan(PIZDEC.x())||std::isnan(PIZDEC.y())) {
                // RCLCPP_INFO(get_logger(), "nope");
                throw std::invalid_argument("hueta");
            }
            
        }
        catch(...) {
            // RCLCPP_INFO(get_logger(), "oh shit");
            return;
        }
        geometry_msgs::msg::Twist res;
        res.linear.x = PIZDEC.y()*0.2;
        res.angular.z = -std::atan2(PIZDEC[0], PIZDEC[1])*0.6 + last_rotate*0.2;
        cmd_vel_pub_->publish(res);
        
        cv_bridge::CvImage out_msg;
        out_msg.header = msg.header;
        out_msg.encoding = "rgb8";
        out_msg.image = output;

        output_pub_->publish(*(out_msg.toImageMsg()));

    }

    void get_depth(const sensor_msgs::msg::Image& msg) {
        if(!depth) {
            depth = new float[msg.width*msg.height];
        }
        std::memcpy(depth, msg.data.data(), sizeof(float)*msg.width*msg.height);
    }

    bool isroad(Vector2i coords) {
        float cds = image_remap(depth, coords);
        if(coords.x() < 0 || coords.x() >= CAM_W || coords.y() < 0 || coords.y() >= CAM_H) {
            return false;
        }
        if(std::isnan(cds)){
            return false;
        }
        Vector2f angles = to_real_angles(coords);
        Vector2f res(
            std::cos(angles.y())*std::sin(angles.x())*cds,
            std::cos(angles.x())*std::cos(angles.y())*cds
        );
        float z = std::cos(angles.x())*std::sin(angles.y())*cds;
        // // RCLCPP_INFO(get_logger(), "%d %d %d %d", coords.x(), coords.y(), std::fabs(z-0.11f) < 0.04, !mask.at<uchar>(479-coords.y(), coords.x()));
        return std::fabs(z-0.10f) < 0.02 && !mask.at<uchar>(479-coords.y(), coords.x());
    }

    Vector2f get_first_pixel() {
        Vector2i l(424, 0);
        Vector2i r(424, 0);

        Vector2i one(1, 0);

        while(l.x() < CAM_W && isroad(l)){
            l += one;
        }
        while(r.x()>=0 && isroad(r)) {
            r -= one;
        }

        l -= one;
        r += one;
        return ((l+r)/2).cast<float>();
    }

    template<class T>
    T image_remap(T* img, Vector2i coords) {
        return img[((CAM_H-coords.y()-1)*CAM_W + coords.x())];
    }

    Vector2f to_real_angles(Vector2i coords) {
        // std::cout << "real angles enter" << std::endl;
        std::fflush(stdout);
        float y = CAM_H - coords.y() - 1;
        float x = coords.x();
        float xx = std::atan(std::tan(CAM_FOV_HOR/2.0f)*(2*x/(CAM_WF-1.0f)-1.0f));
        float yy = CAM_ANGLE + std::atan(std::tan(CAM_FOV_VER/2.0f)*(2*y/(CAM_HF-1.0f)-1.0f));
        // std::cout << "real angles" << xx << " " << yy << std::endl;
        std::fflush(stdout);
        return Vector2f(xx, yy);
    }

    Vector2f img_coords_translate_2_road(Vector2i coords) {
        // std::cout << "translate coords enter " << coords.x() << " " << coords.y() << std::endl;
        float cds = image_remap(depth, coords);
        Vector2f angles = to_real_angles(coords);
        float xx = std::cos(angles.y())*std::sin(angles.x())*cds;
        float yy = std::cos(angles.x())*std::cos(angles.y())*cds;
        Vector2f res(
            xx,
            yy
        );
        
        float z = std::cos(angles.x())*std::sin(angles.y())*cds;
        // std::cout << "translate coords " << xx << " " << yy << " " << z <<  " " << cds << std::endl;
       
            float coef = 0.11/z;
            res *= coef;
        
        return res;
    }

    Vector2f get_fucked(Vector2f coord1 ,Vector2f coord2, int road_t) {
        int s_pt = 5;
        Vector2f dat[2][2]= {
            {Vector2f(424, 0), Vector2f(424, s_pt)},
            {Vector2f(424, 0), Vector2f(424, s_pt)}
        };

        for(int j = 0; j < 2; j++) {
            for(int i = 424; i < 848; i++) {
                if(!isroad(Vector2i(i, j*s_pt))){
                    break;
                }
                dat[1][j].x() = i;
            }

            for(int i = 424; i > 0; i--){
                if(!isroad(Vector2i(i, j*s_pt))) {
                    break;
                }
                dat[0][j].x() = i;
            }
        }
        auto road = get_line(img_coords_translate_2_road(dat[road_t][0].cast<int>()), img_coords_translate_2_road(dat[road_t][1].cast<int>()));
        auto my_line = get_line(coord1, coord2);

        if(road.first == Orientation::Vertical) {
            float x = road.second.at(0);
            if(my_line.first == Orientation::Vertical) {
                return Vector2f(NAN, NAN);
            }
            float y = my_line.second.at(0)*x + my_line.second.at(1);
            return Vector2f(x, y);
        }

        float k1, b1;
        k1 = road.second.at(0);
        b1 = road.second.at(1);

        float k2, b2;
        k2 = my_line.second.at(0);
        b2 = my_line.second.at(1);

        if(k1 == k2) {
            return Vector2f(NAN, NAN);
        }
        float x = (b2-b1)/(k1-k2);
        float y = k1*x+b1;
        return Vector2f(x, y);
    }

    std::pair<Vector2f, Vector2f> check_down(Vector2f coord1, Vector2f coord2) {
        Vector2f c1 = img_coords_translate_2_road(coord1.cast<int>());
        Vector2f c2 = img_coords_translate_2_road(coord2.cast<int>());
        if(coord1.y() == 0) {
            auto n_coord = get_fucked(c1, c2, 0);
            if(std::isnan(n_coord.x())) {
                return std::make_pair(c1, c2);
            }
            return std::make_pair(n_coord, c2);
        }
        if(coord2.y()==0) {
            auto n_coord = get_fucked(c1, c2, 1);
            if(std::isnan(n_coord.x())) {
                return std::make_pair(c1, c2);
            }
            return std::make_pair(c1, n_coord);
        }
        return std::make_pair(c1, c2);
    }
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalPublisher>());
	rclcpp::shutdown();
	return 0;
}