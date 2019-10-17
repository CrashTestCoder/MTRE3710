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
constexpr float min_range = .05;

enum wall { left = 1, right = -1 };

constexpr wall follow = wall::right; // for when zane decides the robot should go the other way...
constexpr float setpoint = pi - pi/2 * follow;
constexpr float setdist = .25;
constexpr float def_speed = .25;
float speed = def_speed;

template<typename T, typename F = void(*)(), typename... Params>
inline void wait(T delay, F unary_predicate = [](Params...) noexcept {}, Params... params)
{
    using namespace std::chrono;
    auto const start = high_resolution_clock().now();
    while(duration_cast<T>(high_resolution_clock().now() - start) < delay) 
        unary_predicate(params...);
}

/*********************************/
/*         program logic         */
/*********************************/

void start_wheelie()
{
    using namespace std::chrono;
    twist = geometry_msgs::Twist();

    twist.linear.x = -1;
    cmd_vel.publish(twist);

    wait(.4s);

    twist.linear.x = 1;
    cmd_vel.publish(twist);

    ros::Duration(.1).sleep();
    
    twist.linear.x = .5;
    cmd_vel.publish(twist);
}

constexpr float get_wheelie_angle(float const dist) noexcept
{
    constexpr float wheel_radius = .02;
    constexpr float lidar_pos_y = .06;
    constexpr float lidar_pos_x = .04;
    float const lidar_hyp = std::sqrt(lidar_pos_x*lidar_pos_x+lidar_pos_y+lidar_pos_y);

    complex const num { wheel_radius, -std::sqrt(dist*dist - wheel_radius*wheel_radius + lidar_hyp*lidar_hyp) };
    complex const den { lidar_hyp, dist };

    complex const theta = -log(-num/den) * complex{ 0, 1 };

    return theta.real;
}

static void correct_pitch(auto const current, auto const target)
{
    if(curent < .2) start_wheelie();

    twist.linear.x = target - current + speed;
}

geometry_msgs::Twist twist;
ros::Publisher cmd_vel;
static void processLaserScan(sensor_msgs::LaserScan::ConstPtr const &scan)
{
    // wheelie
    auto const& back = find_if_not(begin(scan->ranges) + scan->ranges.size()/2, end(scan->ranges),
        [](auto const& data){
            return data < min_range;
        });

    float const robot_pitch = get_wheelie_angle(*back);

    correct_pitch(robot_pitch, pi/3);

    // wall follow

    int fr_size = 10;
    decltype(back) fr_begin, fr_end; // follow range
    if(follow == right) {
        fr_begin = begin(scan->ranges) + scan->ranges.size()/4 - fr_size/2;
        fr_end = fr_begin + fr_size;
    } else {
        fr_begin = begin(scan->ranges) + 3*scan->ranges.size()/4 - fr_size/2;
        fr_end = fr_begin + fr_size;
    }

    auto const& min_dist = min_element(fr_begin, fr_end);
    if((min_dist == fr_begin || min_dist == fr_end) && speed > 0)
        speed -= .01;
    else if(speed < def_speed)
        speed += .01;

    float const wall_angle = scan->angle_increment * ((int)min_dist - fr_size / 2);

    twist.angular.z = wall_angle * follow;

    cmd_vel.publish(twist);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab3");

    ros::NodeHandle n;
    cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &processLaserScan);
    
    ros::spin();

    return 0;
}
