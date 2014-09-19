#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include "ros/ros.h"
using namespace std;

double min_lin_x = 0;
double max_lin_x = 0;
double min_lin_y = 0;
double max_lin_y = 0;
double min_ang = 0;
double max_ang = 0;
ofstream file;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(msg->twist.twist.linear.x >= max_lin_x)
    {
        max_lin_x = msg->twist.twist.linear.x;
    }
    else if (msg->twist.twist.linear.x < min_lin_x)
    {
        min_lin_x = msg->twist.twist.linear.x;
    }
    if(msg->twist.twist.linear.y >= max_lin_y)
    {
        max_lin_y = msg->twist.twist.linear.y;
    }
    else if (msg->twist.twist.linear.y < min_lin_y)
    {
        min_lin_y = msg->twist.twist.linear.y;
    }
    if(msg->twist.twist.angular.z >= max_ang)
    {
        max_ang = msg->twist.twist.angular.z;
    }
    else if(msg->twist.twist.angular.z < min_ang)
    {
        min_ang = msg->twist.twist.angular.z;
    }
    
}

void teleopCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if ((msg->buttons[9]==1)&&(msg->buttons[8])==1)
    {
      	file.open ("/home/skel/velocities.txt", ios::out | ios::binary);
        file<<"Maximum Linear Velocity (x) :"<<max_lin_x<<endl;
        file<<"Minimum Linear Velocity (x) :"<<min_lin_x<<endl;
        file<<"Mayimum Linear Velocity (y) :"<<max_lin_y<<endl;
        file<<"Minimum Linear Velocity (y) :"<<min_lin_y<<endl;
        file<<"Maximum Angular Velocity :"<<max_ang<<endl;
        file<<"Minimum angular velocity :"<<min_ang<<endl;
        file.close();
    }
}

int main(int argc, char *argv[])
{	
    ros::init(argc, argv, "move_script");

	ros::NodeHandle n;
	ros::Duration(0.1).sleep();
	ros::Subscriber sub = n.subscribe("joy", 1, teleopCallback);
    ros::Subscriber vel = n.subscribe("odom",10,odomCallback);
    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
