#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
// %EndTag(MSG_HEADER)%

#include<vector>
#include<algorithm>

using namespace std;
using namespace geometry_msgs;

const float pi = 3.14159265;

vector<float> lidar_data;
float angle_increment;
float angle_min;
float angle_max;
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  lidar_data = scan->ranges;
  angle_increment = scan->angle_increment;
  angle_min = scan->angle_min;
  angle_max = scan->angle_max;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  
  ros::NodeHandle n;
  ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan",10,&processLaserScan);

  ros::Rate loop_rate(2);
  int count = 0;

  double x = .1,y = 0,z = 0,roll = 0,pitch = 0,yaw = 0;
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    
    // find target angle
    auto min_reading = min_element(lidar_data.begin(), lidar_data.end());
    int min_pos = min_reading - lidar_data.begin();
    
    double wall_angle = angle_min + angle_increment * min_pos;

    double setpoint = pi/2;

    wall_angle -= setpoint;
    if(wall_angle < -pi)
      wall_angle += 2*pi;
    wall_angle += setpoint;

    if(wall_angle > setpoint + .1)
      yaw = 1;
    else if( wall_angle < setpoint - .1)
      yaw = -1;
    else yaw = 0;
      

    cout << wall_angle << '\t' << yaw << '\n';

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

    ros::spinOnce();
    loop_rate.sleep();
    ros::Rate(500);
    ++count;
  }

  return 0;
}
