//#include <iostream>
//#include <fstream>
//#include <sstream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "robodev.cpp"
#include <sdc2130_skel/RoboteqDevice.h>
#include <sdc2130_skel/ErrorCodes.h>
#include <sdc2130_skel/Constants.h>
#include "ros/ros.h"
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cmath> 

geometry_msgs::PoseWithCovarianceStamped init_pose;
geometry_msgs::PoseStamped goal_pose;
int sent=1;
int goal_sent=0;

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    //cout<<"gsdgs"<<endl;
    init_pose.pose=msg->pose;
    init_pose.header.frame_id = "/map";
    sent=0;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal_pose.pose=msg->pose;
    goal_pose.header.frame_id="/map";
    goal_pose.header.stamp = ros::Time::now();
    goal_sent=1;
}
int main(int argc, char *argv[])
{	
    ros::init(argc, argv, "init_pose_send");
    ros::NodeHandle n;
    //ros::Subscriber sub4 = n.subscribe("sent_in_pose", 1, initPoseCallback);
	ros::Publisher init_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 5,true);
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5,true);
    ros::Subscriber sub = n.subscribe("initial_pose", 50, initPoseCallback);
    ros::Subscriber sub2 = n.subscribe("goal_pose", 50, goalCallback);
   
   while (ros::ok())
    {	
        
        //cout<<"wow so spin"<<endl;
        ros::Duration(0.2).sleep();
        if (sent==0)
        {
            //cout<<"oeo"<<endl;
            init_pub.publish(init_pose);
            sent=1;
       
        }
        if(goal_sent==1)
        {
            cout<<"pewpewpew"<<endl;
            goal_pub.publish(goal_pose);
            goal_sent=0;
        }
        ros::spinOnce();

        
   } 
    return 0;
}
