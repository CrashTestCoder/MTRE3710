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
constexpr float speed = .25;



/*********************************/
/*         program logic         */
/*********************************/

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

geometry_msgs::Twist twist;
ros::Publisher cmd_vel;
static void processLaserScan(sensor_msgs::LaserScan::ConstPtr const &scan)
{
    
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
