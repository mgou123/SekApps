#include <stdio.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>
#include <chrono>
#include <ctime>

#define NUM_READINGS 1128

using namespace std;

float hok_read[NUM_READINGS] = {};
int REC = 0;
long int timer;
double seconds ,seconds_, seconds2 ;

time_t t;
ros::Time total_cur_time, total_time;
//hokuyo timestamps/streams
ros::Time hok_cur_time, hok_last_time;
ofstream file;
std::stringstream sstream ;
std::string hok_ts, odom_ts;
std::chrono::time_point<std::chrono::system_clock> p1;

//odometry timestamps/streams
ros::Time odom_cur_time, odom_last_time;
ofstream file2;
std::stringstream sstream2 ;
std::chrono::time_point<std::chrono::system_clock> p2;

void hok_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    std::vector<float> ranges = scan->ranges;
	if(REC==1)
	{	
	//TIME INTERVAL STUFF	
		total_cur_time=ros::Time::now();
		hok_cur_time = ros::Time::now();
		seconds=hok_cur_time.toSec()-hok_last_time.toSec();
        if(seconds>0.1)
        {	
			//file<<"Time ";
			p1 = std::chrono::system_clock::now();
			sstream << std::chrono::duration_cast<std::chrono::milliseconds>(
			p1.time_since_epoch()).count() ;
            hok_ts = sstream.str();
            file<<hok_ts<<" ";
			for( std::vector<float>::size_type i = 0; i != ranges.size(); i++) 
			{
				file<<ranges[i]<<" ";
			}
			file<<endl ;
			hok_ts.clear();
			sstream.str(std::string());
			sstream.clear();
			hok_last_time=hok_cur_time;
			hok_cur_time=ros::Time::now();
        }
	}

}

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
	if(REC==1)
	{	
	//TIME INTERVAL STUFF	
		total_cur_time=ros::Time::now();
		odom_cur_time = ros::Time::now();
		seconds2=odom_cur_time.toSec()-odom_last_time.toSec();
        if(seconds2>0.1)
        {	
			//file2<<"Time ";
			p2 = std::chrono::system_clock::now();
			sstream2 << std::chrono::duration_cast<std::chrono::milliseconds>(
			p2.time_since_epoch()).count() ;
            odom_ts = sstream2.str();
            file2<<odom_ts<<" ";
			file2<<odom->twist.twist.linear.x<<" "<<odom->twist.twist.linear.y<<" "<<odom->twist.twist.linear.z<<" "<<odom->twist.twist.angular.x<<" "<<odom->twist.twist.angular.y<<" "<<odom->twist.twist.angular.z<<" "<<endl;
			file2<<endl ;
			odom_ts.clear();
			sstream2.str(std::string());
			sstream2.clear();
			odom_last_time=odom_cur_time;
			odom_cur_time=ros::Time::now();
	
		}
		
	}
}

void callback(const std_msgs::Float64::ConstPtr& msg)
{
    float start = msg->data; //START gia na 3ekinhsei to record
		//X gia na stamathsei
    //printf("Press Start to Start Recording \n");
    		

    if ((REC==1)&&(start==1))
    {	
		cout<<"Stopped Recording Hok\n"<<endl;
        REC = 0;
        //printf("Stopped Recording Skeleton \n");
		ros::shutdown();
    }
    else if (start==1)
    {	
		cout<<"Started Recording Hok\n"<<endl;
		//printf("Started Recording Skeleton\n");
		total_time = ros::Time::now();
        REC = 1;
        ros::Duration(1).sleep();

    }
    
}

int main(int argc, char** argv)
{	
    file.open ("./_laser.data", ios::out | ios::binary);
    file2.open("./odom.data",ios::out | ios::binary);
    ros::init(argc, argv, "recorder");
    ros::NodeHandle n;
    ros::Subscriber gp_in,hok_in,odom_in;
    ros::Publisher turn_off;
    gp_in =n.subscribe("coms",1,callback);
    hok_in = n.subscribe("/scan",1,hok_cb);
    odom_in = n.subscribe("/odom",1,odom_cb);
    while (ros::ok())
    {
        ros::spin();
	}
	file.close();
    file2.close();
	return 0;
}
