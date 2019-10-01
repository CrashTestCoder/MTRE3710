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

#include <vector>
#include <algorithm>
#include <limits>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

using namespace std;
using namespace geometry_msgs;

//#define CoarseMode
#define SPEED 1


/**********************************/
/**** define program constants ****/
/**********************************/
constexpr float pi = 3.14159265;
constexpr float min_range = .05;

enum wall { left = -1, right = 1 };

constexpr wall follow = wall::right; // for when zane decides the robot should go the other way...
constexpr double setpoint = pi - pi/2 * follow;

#ifdef CoarseMode
constexpr double setdist = .2; // .27
#elif SPEED
constexpr double setdist = .3; // .27
#else
constexpr double setdist = .3; // .27
#endif

/**
 * returns the difference between two angles
 * Ex:
 *      pi/2 = rel_angle(3*pi/2, pi);
 */
constexpr double rel_angle(double const& angle, double const& reference)
{
    return atan2(sin(reference-angle), cos(reference-angle));
}



ros::Publisher cmd_vel;
vector<float> lidar_data;
float angle_increment;
float angle_min;
float angle_max;
geometry_msgs::Twist twist;
/**
 * Loads data from lidar and stores it in global variables
 */
static void processLaserScan(sensor_msgs::LaserScan::ConstPtr const &scan)
{
    lidar_data = scan->ranges;
    angle_increment = scan->angle_increment;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;

    // make sure it doesn't track itself
    std::replace_if(lidar_data.begin(), lidar_data.end(), [](float const& data) noexcept {
        return data < min_range;
    }, std::numeric_limits<float>::infinity());
    
    // find wall angle
    auto const& min_reading = std::min_element(lidar_data.begin(), lidar_data.end());
    int const min_pos = min_reading - lidar_data.begin();

    double const wall_angle = angle_min + angle_increment * min_pos;

    // calculate correction angle
    double const yaw_err = rel_angle(setpoint, wall_angle);

    // tendancy to move to a set distance from the wall
    double const y_err = (*min_reading - setdist) * -follow;
    double const find_distance = y_err * y_err * y_err * atan(-y_err) * sin(wall_angle);

    // set ouputs

#ifdef CoarseMode
    twist.linear.x = .3;
    twist.angular.z = twist.linear.x*7*(2.2*yaw_err + 1.7*y_err);
#elif SPEED
    twist.linear.x = 1;
    twist.angular.z = twist.linear.x*6*(1*yaw_err + .8*y_err);
#else
    twist.linear.x = .3;
    twist.angular.z = twist.linear.x*8*(1*yaw_err + .8*y_err);
#endif

    if(yaw_err > 1) twist.linear.x = 0.05;//, twist.angular.z *=3;

    cout << yaw_err << '\n';

    cmd_vel.publish(twist);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab3");

    ros::NodeHandle n;
    cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &processLaserScan);
    
    ros::spin();

    twist = geometry_msgs::Twist();
    cmd_vel.publish(twist);

    return 0;
}
