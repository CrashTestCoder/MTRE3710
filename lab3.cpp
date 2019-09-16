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

using namespace std;
using namespace geometry_msgs;

/**********************************/
/**** define program constants ****/
/**********************************/
constexpr float pi = 3.14159265;
constexpr float min_range = .1;

enum wall { left = 1, right = -1 };

constexpr wall follow = wall::right; // for when zane decides the robot should go the other way...
constexpr double setpoint = pi - pi/2 * follow;
constexpr double setdist = .25;

/*********************************/
/*         program logic         */
/*********************************/
vector<float> lidar_data;
float angle_increment;
float angle_min;
float angle_max;
/**
 * Loads data from lidar and stores it in global variables
 */
static void processLaserScan(sensor_msgs::LaserScan::ConstPtr const &scan)
{
    lidar_data = scan->ranges;
    angle_increment = scan->angle_increment;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
}

/**
 * returns the difference between two angles
 * Ex:
 *      pi/2 = rel_angle(3*pi/2, pi);
 */
static double rel_angle(double const& angle, double const& reference)
{
    return atan2(sin(reference-angle), cos(reference-angle));
}

template<typename T>
static void filter_values_too_small(std::vector<T>& v, T min_val)
{
    // Filter outliers that are too close
    while(1)
    {
        auto& outlier = std::find_if(v.begin(),v.end(),
            [const& min_val](auto const& data) {
                return data < min_range;
            });
        if(outlier == v.end())
            break;

        cout << *outlier << '\n'; // This function is pointless if this is never called
        *outlier = inf;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab3");

    ros::NodeHandle n;
    ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &processLaserScan);

    // These are initialized outside of the loop because ros is bad at initializing things efficiently
    geometry_msgs::Twist twist;
    geometery_msgs::Vector3 linear;
    geometery_msgs::Vector3 angular;
    
    while (ros::ok())
    {
        if(!lidar_data.empty()) // it's empty for the first few iterations for some reason...
        {
            // make sure it doesn't track itself
            filter_values_too_small(lidar_data, min_range);
            
            // find wall angle
            auto const& min_reading = std::min_element(lidar_data.begin(), lidar_data.end());
            int const min_pos = min_reading - lidar_data.begin();

            double const wall_angle = angle_min + angle_increment * min_pos;

            // calculate correction angle
            double const yaw_err = rel_angle(setpoint, wall_angle);

            // tendancy to move to a set distance from the wall
            double const y_err = (*min_reading - setdist) * follow;
            double const find_distance = y_err*y_err*y_err*atan(-y_err) * sin(wall_angle);

            // set ouputs
            linear.x = .5;
            angular.z = 10*yaw_err + 8*find_distance;

            twist.linear = linear;
            twist.angular = angular;

            cmd_vel.publish(twist);
        }

        ros::spinOnce();
    }

    return 0;
}
