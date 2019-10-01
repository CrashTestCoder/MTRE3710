/**
 * Lab 3 - Wall Follower
 * 
 * Authors:
 *      Ryan McHale
 *      Link Reilly
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>

#include "complex.hpp"

#include <vector>
#include <algorithm>
#include <limits>
#include <chrono>

using namespace std;
using namespace geometry_msgs;

/**********************************/
/**** define program constants ****/
/**********************************/
constexpr float pi = 3.14159265;
constexpr float min_range = .015;

enum class wall : int { left = 1, right = -1 };

constexpr wall follow = wall::right; // for when zane decides the robot should go the other way...
constexpr float setpoint = pi - pi/2 * (int)follow;
constexpr float setdist = .25;
constexpr float def_speed = 0;
float speed = def_speed;



/*********************************/
/*         program logic         */
/*********************************/

namespace RM
{
    template<typename T>
    void wait(T delay)
    {
        using namespace std::chrono;
        auto const begin = high_resolution_clock().now();
        while((high_resolution_clock().now() - begin) < delay);
    }
}

geometry_msgs::Twist twist;
ros::Publisher cmd_vel;

void start_wheelie()
{
    using namespace std::chrono;
    twist = geometry_msgs::Twist();

    twist.linear.x = -1;
    cmd_vel.publish(twist);
    ros::spinOnce();

    std::cout << "Back" << std::endl;
    
    RM::wait(200ms);

    twist.linear.x = 0;
    cmd_vel.publish(twist);
    ros::spinOnce();
    std::cerr << "Forward" << std::endl;

    RM::wait(.22s);

    for(;twist.linear.x < .2; twist.linear.x+= .015)
    {
        cmd_vel.publish(twist);
        ros::spinOnce();
        RM::wait(.1s);
    }
    std::cerr << "Less" << std::endl;

    RM::wait(200ms);
}

static void get_wheelie_angle(sensor_msgs::Imu::ConstPtr const& imu)
{
    tf::Matrix3x3 m(quaternion(imu->orientation));
    float const pitch = [=,m]()
    {
        float pitch;
        m.getRPY(nullptr, pitch, nullptr);
        return pitch;
    }();
    twist.linear.x = 5*( - pitch) + speed;
    std::cout << pitch << '\n';
}

static void correct_pitch(auto const current, auto const target)
{
    
    cout<< current << std::endl;
    twist.linear.x = (target - current) + speed;
}

static void processLaserScan(sensor_msgs::LaserScan::ConstPtr const &scan)
{
    // wheelie
    auto const back = find_if_not(begin(scan->ranges) + scan->ranges.size()/2, end(scan->ranges),
        [](auto const& data){
            return data < min_range || isnan(data) || isinf(data);
        });
    
    // wall follow

    int fr_size = 10;
    auto const& fr_begin { [&,scan]{
        if(follow == wall::right) 
            return begin(scan->ranges) + ((1*scan->ranges.size()- 2*fr_size)/4);
        return begin(scan->ranges) + ((3*scan->ranges.size() - 2*fr_size)/4);
    }() };
    auto const& fr_end = fr_begin + fr_size;

    auto const& min_dist = min_element(fr_begin, fr_end);
    if((min_dist == fr_begin || min_dist == fr_end) && speed > 0)
        speed -= .01;
    else if(speed < def_speed)
        speed += .01;

    float const wall_angle = scan->angle_increment * ((min_dist - fr_begin) - fr_size / 2);

    twist.angular.z = wall_angle * (int)follow;

    cmd_vel.publish(twist);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab3");

    ros::NodeHandle n;
    cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &processLaserScan);
    ros::Subscriber imu = n.subscribe<sensor_msgs::Imu>("/imu", 10, &get_wheelie_angle);
    
    //start_wheelie();
    ros::spin();

    return 0;
}
/*
constexpr float get_wheelie_angle(float const dist) noexcept
{
    constexpr float wheel_radius = .02;
    constexpr float lidar_pos_y = .06;
    constexpr float lidar_pos_x = .04;
    float const lidar_hyp = std::sqrt(lidar_pos_x*lidar_pos_x+lidar_pos_y+lidar_pos_y);

    rm::complex const num { wheel_radius, -std::sqrt(dist*dist - wheel_radius*wheel_radius + lidar_hyp*lidar_hyp) };
    rm::complex const den { lidar_hyp, dist };

    rm::complex const theta = -log(num/den) * rm::complex{ 0, 1 };

    return theta.real;
}
*/
