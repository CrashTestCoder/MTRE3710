#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
// %EndTag(MSG_HEADER)%

#include <vector>
#include <algorithm>
#include <chrono>

using namespace std;
using namespace std::chrono;
using namespace geometry_msgs;

const float pi = 3.14159265;

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
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;
    ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &processLaserScan);

    ros::Rate loop_rate(2);
    int count = 0;

    double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
    double pre_x = .1, pre_y = 0, pre_z = 0, pre_roll = 0, pre_pitch = 0, pre_yaw = 0;
    
    double yaw_err, y_err;


    while (ros::ok())
    {
        if(!lidar_data.empty())
        {
            const wall follow = wall::right;
            const double setpoint = pi - pi/2 * follow;
            const double setdist = .25;

            geometry_msgs::Twist msg;
            
            // find target angle
            const auto min_reading = min_element(lidar_data.begin(), lidar_data.end());
            const int min_pos = min_reading - lidar_data.begin();

            const double wall_angle = angle_min + angle_increment * min_pos;

            // calculate correction angle
            yaw_err = rel_angle(setpoint, wall_angle);

            // tendancy to move to a set distance from
            y_err = (*min_reading - setdist) * follow;
            double find_distance = y_err*y_err*y_err*atan(-y_err) * sin(wall_angle);

            // set ouputs
            yaw = 10*yaw_err+ 8*find_distance;
            x = .5;

            cout << y_err << '\t' << yaw << '\t' << yaw_err << '\n';

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
        //loop_rate.sleep();
        //ros::Rate(5000);
        ++count;
    }

    geometry_msgs::Twist msg;
    
    // Create velocity vector
    Vector3 linear;
    linear.x = 0;
    linear.y = 0;
    linear.z = 0;
    msg.linear = linear;

    // Create angular velocity vector
    Vector3 angular;
    angular.x = 0;
    angular.y = 0;
    angular.z = 0;
    msg.angular = angular;

    cmd_vel.publish(msg);

    return 0;
}
