#include "helper.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>

// SETINGS
const uint32_t ITER_COUNT = 10;
const float VEC_SIZE = 5;
const float VEC_FADING = 0.6;
const float MAX_ANGLE = 20*M_PI/180;
const float ROAD_HEIGHT = 0.105f;
const float ROAD_ERR = 0.02f;
//


// CAM CHARACTERISTICS 
const float CAM_ANGLE = 0.11;
const uint32_t CAM_H = 480;
const uint32_t CAM_W = 848;
const float CAM_FOV_HOR = 1.51843645;
const float CAM_FOV_VER = 2 * std::atan(std::tan(CAM_FOV_HOR/2)*CAM_H/CAM_W);
//

uint8_t white_l[3] = {0, 0, 230};
uint8_t white_h[3] = {179, 70, 255};

uint8_t yellow_l[3] = {10, 100, 180};
uint8_t yellow_h[3] = {40, 255, 255};

vec<float> to_real_angles(vec<int> coords)
{
    coords.y = CAM_H - coords.y - 1;
    vec<float> res;

    res.x = std::atan(std::tan(CAM_FOV_HOR/2.0f)*(2.0f*(float)coords.x/((float)CAM_W-1.0f)-1.0f));
    res.y = CAM_ANGLE + std::atan(std::tan(CAM_FOV_VER/2.0f)*(2.0f*(float)coords.y/((float)CAM_H-1.0f)-1.0f));
    
    return res;
}


bool inRange(RGB8 pixel, uint8_t lower[3], uint8_t upper[3]) {
    bool h = pixel.r > lower[0] && pixel.r < upper[0];
    bool s = pixel.g > lower[1] && pixel.g < upper[1];
    bool l = pixel.b > lower[2] && pixel.b < upper[2];
    return h && s &&l;
}



point3 get_line(vec<float> f_p,vec<float> s_p)
{
    point3 res;
    res.x = 1; //Horizontal
    if (abs(f_p.x-s_p.x)<0.01)
    {
        res.x = 0; //Vertical
        res.y = (f_p.x+s_p.x)/2;
        return res;        
    }
    res.y = (f_p.y-s_p.y)/(f_p.x-s_p.x);
    res.z = f_p.y-f_p.x*res.y;
    return res;
}

template <typename T>
T image_remap(const T* image,vec<int> ind)
{
    return image[(CAM_H-ind.y-1)*CAM_W+ind.x];
}

class IIS
{
	float last_lin = 0;
	float last_ang = 0;
	int walls_cnt=1;
    bool is_wall = false;
	float k_dif=0.1;
    
public: 
    sensor_msgs::msg::LaserScan* lidar;
	float* depth = nullptr;
	point3* cloud = nullptr;
    vec<float> pos;
    float angle;
    float cringe = 0.5;
    float needed_angle=NAN;
    cv::Mat image;
    cv::Mat mask;
    rclcpp::Logger* logger;
    bool is_constract=false;
    
    cv_bridge::CvImagePtr show;

    inline float float_lidar(float angles)
    {
        auto float_ind = angles/lidar->angle_increment;
        if (float_ind<0)
            float_ind += lidar->ranges.size();

        int ind = std::floor(float_ind);
        return lidar->ranges[ind]*(float_ind-ind) + (ind+1-float_ind)*lidar->ranges[ind+1];
    }

    inline bool isroad(vec<int> coords)
    {
        
        // RCLCPP_INFO(this->get_logger(),"%f",image_remap(cloud,idx).z);
        if (0<=coords.x && coords.x<CAM_W && 0<=coords.y && coords.y<CAM_H)
        {
            auto angles = to_real_angles(coords);
            
            float cds = image_remap(depth,coords);

            auto tan_b = tan(angles.y);
            auto tan_a = tan(angles.x);
            
            float y = cds/sqrt(1+tan_b*tan_b + tan_a*tan_a);
            float z = y*tan_b;

            
            return abs(z-ROAD_HEIGHT)<ROAD_ERR && !mask.at<uchar>(479-coords.y, coords.x);
        }
            
        return false;
    }

    inline bool float_isroad(vec<float> coords)
    {
        vec<int> int_coords((int)floor(coords.x),(int)floor(coords.y));

        if ((float)int_coords.x == coords.x)
        {
            if ((float)int_coords.y==coords.y)
                return isroad(int_coords);
            
            return isroad(int_coords) && isroad(vec<int>(int_coords.x,int_coords.y+1));
        } 
        if ((float)int_coords.y==coords.y)
            return isroad(int_coords) && isroad(vec<int>(int_coords.x+1,int_coords.y));
        
        return isroad(int_coords) && isroad(vec<int>(int_coords.x,int_coords.y+1)) && isroad(vec<int>(int_coords.x+1,int_coords.y)) && isroad(vec<int>(int_coords.x+1,int_coords.y+1));
    }

    vec<float> img_coords_translate_2_road(vec<int> coords)
    {
        auto angles = to_real_angles(coords);
        vec<float> res;
        float cds = image_remap(depth,coords);


        auto tan_b = tan(angles.y);
        auto tan_a = tan(angles.x);
        
        res.y = cds/sqrt(1+tan_b*tan_b + tan_a*tan_a);
        res.x = res.y*tan_a;
        float z = res.y*tan_b;

        // auto pt = image_remap(cloud,coords);
        // RCLCPP_INFO(this->get_logger(),"%f\n",z);
      
        float coef = ROAD_HEIGHT/z;

        res.x *= coef;
        res.y *= coef;
    
        
        // res.x = pt.y;
        // res.y = pt.x;
        
        return res;
    }

    vec<float> float_img_translate(vec<float> coords)
    {
        vec<int> int_coords((int)floor(coords.x),(int)floor(coords.y));
        
        auto l_d = img_coords_translate_2_road(int_coords);
        if ((float)int_coords.x == coords.x && (float)int_coords.y==coords.y)
            return l_d;
        
        if ((float)int_coords.y == coords.y)
        {
            auto r_d = img_coords_translate_2_road(vec<int>(int_coords.x+1,int_coords.y));
            return l_d*(int_coords.x+1-coords.x)+r_d*(coords.x - int_coords.x);
        }

        if ((float)int_coords.x == coords.x)
        {
            auto l_u = img_coords_translate_2_road(vec<int>(int_coords.x,int_coords.y+1));
            return l_d*(int_coords.y+1-coords.y) + l_u*(coords.y-int_coords.y);
        }

        auto l_u = img_coords_translate_2_road(vec<int>(int_coords.x,int_coords.y+1));
        auto r_d = img_coords_translate_2_road(vec<int>(int_coords.x+1,int_coords.y));
        auto r_u = img_coords_translate_2_road(vec<int>(int_coords.x+1,int_coords.y+1));

        auto f1 = l_d*(int_coords.x+1-coords.x)+ r_d*(coords.x - int_coords.x);
        auto f2 = l_u*(int_coords.x+1-coords.x)+ r_u*(coords.x - int_coords.x);

        return f1*(int_coords.y+1-coords.y) +f2*(coords.y-int_coords.y);
    }

    vec<float> road_coords_2_img(vec<float> coords)
    {
        // std::cout<<"COORDS WTF "<<coords.x<<" "<<coords.y<<" "<<tan(CAM_FOV_HOR/2)<<std::endl;
        auto i = (coords.x/(coords.y*std::tan(CAM_FOV_HOR/2.0f))+1.0f)/2.0f*((float)CAM_W-1.0f);
        auto j = ((float)CAM_H-1.0f)/2.0f*(std::tan(std::atan2(ROAD_HEIGHT,coords.y)-CAM_ANGLE)/std::tan(CAM_FOV_VER/2.0f)+1.0f);
        j = CAM_H - j -1;
        // std::cout<<"COORDS WTF "<<i<<" "<<j<<std::endl;
        return vec<float>(i,j);
    }

    vec<float> get_intersection(vec<float> coord1,vec<float> coord2,int road)
    {
        uint32_t s_pt = 5;
        vec<int> road_coords[2] = {vec<int>(424,0),vec<int>(424,s_pt)};
        for (int j = 0;j<1;j++)
            for (int i = CAM_W/2;0<=i && i<CAM_W && isroad(vec<int>(i,0));i+=2*road-1)
                road_coords[j].x = i;

        auto road_line = get_line(img_coords_translate_2_road(road_coords[0]),img_coords_translate_2_road(road_coords[1]));
        auto my_line = get_line(coord1,coord2);

        vec<float> res;
        if (road_line.x == 0.0f) //VERTICAL
        {   
            res.x = road_line.y;
            if (my_line.x==0.0f)
                return vec<float>(NAN,NAN);
            res.y = my_line.y*res.x + my_line.z;
            return res;
        }

        float k1 = road_line.y,b1 = road_line.z;
        float k2 = road_line.y,b2 = my_line.z;

        if (k1==k2)
            return vec<float>(NAN,NAN);

        res.x = (b2-b1)/(k1-k2);
        res.y = k1*res.x + b1;
        return res;
    }

    std::vector<vec<float>> check_down(vec<float> coord1,vec<float> coord2)
    {
        std::vector<vec<float>> res(2);
        // std::cout<<"GULI GILU1"<<coord1<<std::endl;
        res[0] = float_img_translate(coord1);
        // std::cout<<"GULI GILU2"<<coord2<<std::endl;
        res[1] = float_img_translate(coord2);
        
        if (coord1.y == 0)
        {
            auto n_coord = get_intersection(res[0],res[1],0);
            if (std::isnan(n_coord.x))
                return res;
            res[0] = n_coord;
            return res;            
        }
        if (coord2.y==0)
        {
            auto n_coord = get_intersection(res[0],res[1],1);
            if (std::isnan(n_coord.x))
                return res;
            res[1] = n_coord;
            return res;         
        }
        return res;
    }

    point3 draw_line(vec<float> nv,vec<float> p)
    {
        point3 res; 
        if (nv.y!=0 && abs(nv.x/nv.y)<1)
        {
            res.x = 0; // Vecrtical
            res.y = nv.x/nv.y;
            res.z = -nv.x/nv.y*p.y+p.x;
            return res;
        }
        res.x = 1;
        res.y = nv.y/nv.x;
        res.z = -nv.y/nv.x*p.x + p.y;
        return res;
    }

    vec<float> find_first_pixel()
    {
        vec<int> l(424,0);
        vec<int> r(424,0);
        while (l.x < 848 && isroad(l))
            l.x++; 

        while (r.x >=0 && isroad(r))
                    r.x--;
                
        l.x--;
        r.x++;

        auto smdf = road_coords_2_img((float_img_translate(l)*cringe+float_img_translate(r)*(1-cringe)));
        return vec<float>(smdf.x,0);
    }
    
    vec<float> stupid_work()
    {
        vec<float> res;
        if (walls_cnt >=2)
            return res;
        
        if (lidar->ranges[0]<0.48)
            is_wall = true;
        
        if (is_wall)
        {
            RCLCPP_INFO(*logger,"STENA DETECTED %d",walls_cnt);
            res.x = 0;
            res.y = 1;
            res = res.rotate(-(walls_cnt*2-1)*(70.0f)/180*M_PI);
            // cringe = 0.5 - (walls_cnt*2-1)*0.05;
            // std::cout<<"SUKA TAM STENA POVORACHIVAY BLYAT'"<<res<<" "<<is_wall<<std::endl;
        }
        
        if (lidar->ranges[0]>0.6 && is_wall)
        {
            is_constract = false;
        }
            
        return res;
    }


    vec<float> mrv()
    {
        if (is_constract)
        {
            auto stupid = stupid_work();
            if (stupid.y!=0)
                return stupid;
        }
        

        cv::Mat mat(480, 848, CV_8UC1, cv::Scalar(0));
        cv::Mat wmask, ymask;
        cv::inRange(image, cv::Scalar(0, 0, 230), cv::Scalar(179, 70, 255), wmask);
        cv::inRange(image, cv::Scalar(10, 100, 180), cv::Scalar(40, 255, 255), ymask);
        mask = (wmask | ymask);

        // cv::imwrite("hello_darkness my old friend.png", mask);
        std::vector<vec<float>> points;
        std::vector<vec<float>> vectors;

        points.push_back(find_first_pixel());
        vectors.push_back(vec<float>(0,VEC_SIZE));
        
        auto perp = draw_line(vec<float>(vectors.back().y,-vectors.back().x),points.back() + vectors.back());
        // std::cout<<"FISRT POINT VEC "<<points.back().x<<" "<<points.back().y<<std::endl;
        
        for (int iter=0;iter<ITER_COUNT;iter++)
        {
            auto next_point = points.back()+vectors.back();
            auto now_point = points.back();
            auto now_vec = vectors.back();
            vec<float> left,right;

            // std::cout<<iter<<std::endl;

            if (perp.x == 1)
            { //Horizontal
                right.y = next_point.y;
                for (float i=next_point.x;i<CAM_W;i+=1)
                {
                    auto pt = i*perp.y+perp.z;
                    if (!float_isroad(vec<float>(i,pt)))
                        break;
                    right.x = i;
                    right.y = pt;
                    
                    //show->image.ptr<RGB8>(479-i)[(int)pt]= {255,0,255};
                    show->image.at<RGB8>(479 - (int)pt, (int)i) = {255, 0, 255};
                }

                left.y = next_point.y;
                for (float i=next_point.x;i>=0;i-=1)
                {
                    auto pt = i*perp.y+perp.z;
                    if (!float_isroad(vec<float>(i,pt)))
                        break;
                    left.x = i;
                    left.y = pt;
                    show->image.at<RGB8>(479 - (int)pt, (int)i) = {255, 0, 255};
                }                 
            }  
            else
            { // Vertical
                right.x = next_point.x;
                for (float i=next_point.y;i<CAM_H;i+=1)
                {
                    auto pt = i*perp.y+perp.z;
                    if (!float_isroad(vec<float>(pt,i)))
                        break;
                    right.x = pt;
                    right.y = i;
                    show->image.at<RGB8>(479 - (int)i, (int)pt) = {255, 0, 255};
                }

                left.x = next_point.x;
                for (float i=next_point.y;i>=0;i-=1)
                {
                    auto pt = i*perp.y+perp.z;
                    if (!float_isroad(vec<float>(pt,i)))
                        break;
                    left.x = pt;
                    left.y = i;
                    show->image.at<RGB8>(479 - (int)i, (int)pt) = {255, 0, 255};
                }
                if (left.x>right.x)
                    std::swap(left,right);                       
            }
            // std::cout<<iter<<" "<<float_isroad(left)<<"     "<<float_isroad(right)<<std::endl;
            // std::cout<<"NOT REAL "<<left<<" "<<right<<std::endl;
            auto new_coords = check_down(left,right);
            left = new_coords[0];
            right = new_coords[1];
            // std::cout<<iter<<std::endl;
            // std::cout<<"REAL "<<left<<" "<<right<<std::endl;
            
            auto p_new = road_coords_2_img((left*cringe+right*(1-cringe)));
            auto n_w = vec<float>(p_new - now_point);
            if (std::isnan(n_w.x) || std::isnan(n_w.y) || n_w.len()==0)
                return vec<float>(NAN,NAN);
            
            n_w = n_w.normalize()*VEC_SIZE;
            // std::cout<<"DO PIZDECA "<<n_w.x<<" "<<n_w.y<<std::endl;
            n_w  = n_w*VEC_FADING+now_vec*(1-VEC_FADING);
            // std::cout<<"RESULT VECTOR "<<n_w<<std::endl;
            // auto angle = std::acos((n_w*now_vec)/(VEC_SIZE*VEC_SIZE));
            // std::cout<<"ANGLE "<<angle<<std::endl;
            // if (abs(angle)>MAX_ANGLE)
            //     n_w = vec<float>(now_vec.x*cos(MAX_ANGLE)-now_vec.y*sin(MAX_ANGLE),now_vec.x*sin(MAX_ANGLE)+now_vec.y*cos(MAX_ANGLE))*-sign(angle);

            points.push_back(n_w+now_point);
            if (points.back().y<0)
                return vec<float>(NAN,NAN);
            // std::cout<<"NEXT POINT "<<points.back().x<<" "<<points.back().y<<std::endl;
            if (points.back().x>2 && points.back().y>2)
            {
                for (int i =-2;i<3;i++)
                {
                    for (int j = -2;j<3;j++)
                    {   
                        show->image.at<RGB8>(479 - (i+(int)round(points.back().y)), (j+(int)round(points.back().x))) = {255, 0, 0};
                    }
                        
                }
            }
            
            vectors.push_back(n_w);

            auto perp_vec = float_img_translate(points.back()) - float_img_translate(points[points.size()-2]);
            
            auto buff = perp_vec;
            // std::cout<<"NOT PERP VEC "<<perp_vec.x<<" "<<perp_vec.y<<std::endl;
            //std::cout<<"LEFT VEC "<<((left+right)/2).x<<" "<<((left+right)/2).y<<std::endl;
            //std::cout<<"NEW VEC "<<n_w.x<<" "<<n_w.y<<std::endl;
            perp_vec.x = buff.y;
            perp_vec.y = -buff.x;

            // perp_vec= perp_vec.normalize();
            // std::cout<<"PERP VEC "<<perp_vec.x<<" "<<perp_vec.y<<std::endl;
            perp_vec = perp_vec + float_img_translate(points.back());
            // std::cout<<"PERP VEC "<<perp_vec.x<<" "<<perp_vec.y<<std::endl;
            // std::cout<<"LAST POINT VEC "<<points.back().x<<" "<<points.back().y<<std::endl;
            // std::cout<<"IMG COORDS "<<road_coords_2_img(perp_vec).x<<" "<<road_coords_2_img(perp_vec).y<<std::endl;
            perp_vec = road_coords_2_img(perp_vec)-points.back();
            // std::cout<<iter<<n_w<<perp_vec<<std::endl;
            // std::cout<<"PERP VEC IN COORDS "<<perp_vec.x<<" "<<perp_vec.y<<std::endl;
            perp = draw_line(perp_vec,points.back() + vectors.back());

            // std::cout<<"VERTICAL "<<perp.x<<" "<<perp.y<<" "<<perp.z<<std::endl;
            

        }

        
        
        auto PIZDEC = float_img_translate(points.back())-float_img_translate(vec<float>(424,0));
        if (std::isnan(PIZDEC.x) || std::isnan(PIZDEC.y))
            return vec<float>(NAN,NAN);
        
        return PIZDEC.normalize();
    }

    
    vec<float> random_forest()
    {
        if (std::isnan(needed_angle))
        {
            needed_angle = angle+45.0f/180*M_PI;
            if (needed_angle>M_PI)
                needed_angle-=M_PI;
        }
        
            
        
        vec<float> shwed(1,0);
        shwed = shwed.rotate(needed_angle-angle);

        if (abs(needed_angle-angle)<0.1)
            return shwed;
        return mrv();

    }

    vec<float> egor_letov()
    {
        //Ubivat' ubivat' ubivat' ...

        float max_lidar_angle = 30.0f/2/180*M_PI;
        float max_distanse = 0.4;

        auto smth = (int)(max_lidar_angle/lidar->angle_increment);

        vec<float> res(0,1);
        for (int i = 0;i<smth;i++)
            if (lidar->ranges[i]/cos(lidar->angle_increment*i) < max_distanse)
                res.y = 0;

        for (int i = 0;i<smth;i++)
            if (lidar->ranges[lidar->ranges.size()-i]/cos(lidar->angle_increment*i) < max_distanse)
                res.y = 0;

        return res;
    }

};