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

constexpr float pi = 3.14159265;
constexpr float min_range = .1;

enum wall { left = 1, right = -1 };

vector<float> lidar_data;
float angle_increment;
float angle_min;
float angle_max;
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan)
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
double rel_angle(double angle, double reference)
{
    return atan2(sin(reference-angle), cos(reference-angle));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab3");

    ros::NodeHandle n;
    ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &processLaserScan);

    double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
    double yaw_err, y_err;

    while (ros::ok())
    {
        if(!lidar_data.empty()) // it's empty for the first few iterations for some reason...
        {
            const wall follow = wall::right; // for when zane decides the robot should go the other way...
            const double setpoint = pi - pi/2 * follow;
            const double setdist = .25;

            geometry_msgs::Twist msg;

            // Filter outliers that are too close
            while(1)
            {
                auto& outlier = std::find_if(lidar_data.begin(),lidar_data.end(),[const& min_range](const auto& data) {
                    return data < min_range;
                });
                if(outlier == lidar_data.end())
                    break;
                else 
                    *outlier = inf;
            }
            
            // find target angle
            const auto& min_reading = min_element(lidar_data.begin(), lidar_data.end());
            const int min_pos = min_reading - lidar_data.begin();

            const double wall_angle = angle_min + angle_increment * min_pos;

            // calculate correction angle
            yaw_err = rel_angle(setpoint, wall_angle);

            // tendancy to move to a set distance from the wall
            y_err = (*min_reading - setdist) * follow;
            const double find_distance = y_err*y_err*y_err*atan(-y_err) * sin(wall_angle);

            // set ouputs
            yaw = 10*yaw_err+ 8*find_distance;
            x = .5;

            // Create velocity vector
            Vector3 linear;
            linear.x = x;
            linear.y = y;
            linear.z = z;
            msg.linear = linear;

            // Create angular velocity vector
            Vector3 angular;
            angular.x = roll;
            angular.y = pitch;
            angular.z = yaw;
            msg.angular = angular;

            cmd_vel.publish(msg);
        }

        ros::spinOnce();
    }

    return 0;
}
