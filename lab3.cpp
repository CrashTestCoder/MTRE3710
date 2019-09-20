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

using namespace std;
using namespace geometry_msgs;

/**********************************/
/**** define program constants ****/
/**********************************/
constexpr float pi = 3.14159265;

enum wall { left = 1, right = -1 };

constexpr wall follow = wall::right; // for when zane decides the robot should go the other way...
constexpr float setpoint = pi - pi/2 * follow;
constexpr float setdist = .25;

/**
 * returns the difference between two angles
 * Ex:
 *      pi/2 = rel_angle(3*pi/2, pi);
 */
constexpr auto rel_angle(auto const& angle, auto const& reference)
{
    return atan2(sin(reference-angle), cos(reference-angle));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab3");

    ros::NodeHandle n;

    ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    sensor_msgs::LaserScan::ConstPtr lidar;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, [&lidar](auto const& scan){
        lidar = scan;
    });

    // This is initialized outside of the loop because ros is bad at initializing things efficiently
    geometry_msgs::Twist twist;
    
    while (ros::ok())
    {
        if(! lidar->ranges.empty() ) // it's empty for the first few iterations for some reason...
        {
            // find wall angle
            auto const& min_reading = std::min_element(begin(lidar->ranges), end(lidar->ranges));
            int const min_pos = min_reading - lidar_data.begin();

            float const wall_angle = lidar->angle_min + lidar->angle_increment * lidar->min_pos;

            // calculate correction angle
            float const yaw_err = rel_angle(setpoint, wall_angle);

            // tendancy to move to a set distance from the wall
            float const y_err = (*min_reading - setdist) * follow;
            float const find_distance_correction = y_err*y_err*y_err*atan(-y_err) * sin(wall_angle);

            // set ouputs
            twist.linear.x = .5;
            twist.angular.z = 10*yaw_err + 8*find_distance_correction;

            cmd_vel.publish(twist);
        }

        ros::spinOnce();
    }

    return 0;
}
