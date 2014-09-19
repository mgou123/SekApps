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

geometry_msgs::PoseWithCovarianceStamped init_pose_;
geometry_msgs::PoseWithCovarianceStamped map_pose_;
geometry_msgs::PoseStamped goal_pose_;
int sent_ = 1;
int goal_sent_ = 0;
int map_sent_ = 0;

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_)
{
    cout<<"gsdgs"<<endl;
    init_pose_.pose = msg_->pose;
    init_pose_.header.frame_id = "/map";
    sent_ = 0;
}

void mapPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_)
{
    cout<<"gsdgs"<<endl;
    map_pose_.pose = msg_->pose;
    map_pose_.header.frame_id = "/map";
    map_sent_ = 0;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_)
{
    goal_pose_.pose = msg_->pose;
    goal_pose_.header.frame_id = "/map";
    goal_pose_.header.stamp = ros::Time::now();
    goal_sent_ = 1;
}
int main(int argc, char *argv[])
{	
    ros::init(argc, argv, "init_pose_send");
    ros::NodeHandle n_;
	ros::Publisher init_pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 5, true);
    ros::Publisher goal_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, true);
    ros::Publisher map_pose_pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 5, true);
    ros::Subscriber sub_ = n_.subscribe("initial_pose", 50, initPoseCallback);
    ros::Subscriber sub2_ = n_.subscribe("goal_pose", 50, goalCallback);
    ros::Subscriber sub3_ = n_.subscribe("amcl_pose", 50, mapPoseCallback);
   
   while (ros::ok())
    {	
        
        //cout<<"wow so spin"<<endl;
        ros::Duration(0.2).sleep();
        if (sent_ == 0)
        {
            //cout<<"oeo"<<endl;
            init_pub_.publish(init_pose_);
            sent_ = 1;
       
        }
        if (map_sent_ == 0)
        {
            
        }
        if(goal_sent_ == 1)
        {
            cout<<"pewpewpew"<<endl;
            goal_pub_.publish(goal_pose_);
            goal_sent_ = 0;
        }
        ros::spinOnce();

        
   } 
    return 0;
}
