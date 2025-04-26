#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <memory>
#include <SFML/Graphics.hpp>
#include <chrono>
#include <iostream>
#include <thread>

using namespace std;
using namespace chrono;
using namespace std::chrono_literals;

static constexpr double kEarthRadius = 6378137.0;                 // 平均地球半径，单位：m
static constexpr double kDeg2Rad     = M_PI / 180.0;              // 角度转弧度

double car_x;
double car_y;

std::vector<std::pair<double,double>> quanjulujing;

void xianchenghanshu()   //新线程的开始函数，相当于新线程的main
{
     // 记录上一次刷新时刻
    auto lastTime = std::chrono::steady_clock::now();
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "全局路径");
    while (1) 
    {
        // 处理事件
        sf::Event event;
        while (window.pollEvent(event)) 
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }
       
        // 获取当前时间
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime);
        
        
        if(elapsed.count() % 10==0)
        {
            
           window.clear();   //先清除屏幕，避免重复画

           

           


           // 绘制相邻锥桶之间的连线函数
           auto drawLinesBetweenPoints = [&window](const std::vector<std::pair<double, double>>& points, sf::Color color) {
               for (size_t i = 1; i < points.size(); ++i)
               {
                   sf::Vertex line[] =
                   {
                       sf::Vertex(sf::Vector2f((points[i - 1].first + 20) * 25, (-points[i - 1].second + 20) * 25), color),
                       sf::Vertex(sf::Vector2f((points[i].first + 20) * 25, (-points[i].second + 20) * 25), color)
                   };
                   window.draw(line, 2, sf::Lines);
               }
               };

           

           // 绘制红锥桶的点和连线
        //    for (const auto& num : ditured)
        //    {
        //        sf::CircleShape point(2);
        //        point.setFillColor(sf::Color::Red);
        //        point.setPosition((num.first / 5 + 5) * 25, (-num.second / 5 + 30) * 25);
        //        window.draw(point);
        //    }
          

        //    // 绘制蓝锥桶的点和连线
        //    for (const auto& num : ditublue)
        //    {
        //        sf::CircleShape point(2);
        //        point.setFillColor(sf::Color::Blue);
        //        point.setPosition((num.first / 5 + 5) * 25, (-num.second / 5 + 30) * 25);
        //        window.draw(point);
        //    }
           
           
           
         
           // 绘制贝塞尔曲线的辅助函数
           auto drawBezierCurve = [&window](const std::vector<std::pair<double, double>>& points, sf::Color color) {
               for (size_t i = 1; i < points.size(); ++i) {
                   sf::Vertex line[] = {
                       sf::Vertex(sf::Vector2f((points[i - 1].first + 20) * 25, (-points[i - 1].second + 20) * 25), color),
                       sf::Vertex(sf::Vector2f((points[i].first + 20) * 25, (-points[i].second + 20) * 25), color)
                   };
                   window.draw(line, 2, sf::Lines);
               }
               };


           // 绘制全局曲线的辅助函数
           auto drawCurve = [&window](const std::vector<std::pair<double, double>>& points, sf::Color color) {
               for (size_t i = 1; i < points.size(); ++i) {
                   sf::Vertex line[] = {
                       sf::Vertex(sf::Vector2f((points[i - 1].first+5 ) * 25, (-points[i - 1].second + 20) * 25), color),
                       sf::Vertex(sf::Vector2f((points[i].first+5 ) * 25, (-points[i].second + 20) * 25), color)
                   };
                   window.draw(line, 2, sf::Lines);
               }
               };


           // 绘制贝塞尔曲线的辅助函数
           auto drawdituCurve = [&window](const std::vector<std::pair<double, double>>& points, sf::Color color) {
               for (size_t i = 1; i < points.size(); ++i) {
                   sf::Vertex line[] = {
                       sf::Vertex(sf::Vector2f((points[i - 1].first / 5 + 5) * 25, (-points[i - 1].second / 5 + 30) * 25), color),
                       sf::Vertex(sf::Vector2f((points[i].first / 5 + 5) * 25, (-points[i].second / 5 + 30) * 25), color)
                   };
                   window.draw(line, 2, sf::Lines);
               }
               };


            drawdituCurve(quanjulujing, sf::Color::White);
            
            window.display();

            
        }
        
    }
}


class MapNode
{
public:
    MapNode() : gps_initialized_(false)
    {
        ros::NodeHandle nh;

        car_x=0;
        car_y=0;

        gps_sub_ = nh.subscribe("/fix", 20, &MapNode::gpsCallback, this);

        // 订阅视觉节点发布的左侧数据
        zhuitong_subscription_ = nh.subscribe("zhuitong_topic", 10, &MapNode::zhuitongCallback, this);
       
    }

private:
void zhuitongCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
   
    
    // 检查数据总数是否为43的倍数
    if (msg->data.size() % 3 != 0) {
        ROS_ERROR("接收到的数据个数不是3的倍数!");
        return;
    }

    // 每4个数据为一组：第一个为标志位，然后是centx，centy，z
    for (size_t i = 0; i < msg->data.size(); i += 4) {
        int flag = static_cast<int>(msg->data[i]);
        double x = msg->data[i + 1];  
        double y = msg->data[i + 2];

    }
}
    /** NavSatFix 回调 —— 把经纬度转换为 ENU 并计算水平距离 **/
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        double lat = msg->latitude;      // 当前纬度 (°)
        double lon = msg->longitude;     // 当前经度 (°)

        if (!gps_initialized_)
        {
            ref_lat_ = lat;
            ref_lon_ = lon;
            cos_ref_lat_ = std::cos(ref_lat_ * kDeg2Rad);
            gps_initialized_ = true;
            
            return;
        }

        /* -------- Δ纬度、Δ经度 -> ENU (m) -------- */
        double d_lat_rad = (lat - ref_lat_) * kDeg2Rad;
        double d_lon_rad = (lon - ref_lon_) * kDeg2Rad;

        double east  = kEarthRadius * d_lon_rad * cos_ref_lat_;   // E 方向 (m)
        double north = kEarthRadius * d_lat_rad;                  // N 方向 (m)
        car_x=east;

        car_y=north;

        quanjulujing.push_back({car_x,car_y});

    }

    /* -------- 成员变量 -------- */
    bool   gps_initialized_;
    double ref_lat_, ref_lon_, cos_ref_lat_;
    ros::Subscriber gps_sub_;
    ros::Subscriber zhuitong_subscription_;
    
};

int main(int argc, char** argv)
{
    // 创建一个线程来运行 xianchenghanshu()
    std::thread drawThread(xianchenghanshu);
    drawThread.detach();
    ros::init(argc, argv, "map_node");
    MapNode node;
    ros::spin();
    return 0;
}
