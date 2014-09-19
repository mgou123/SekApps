#include <stdio.h>
#include <string.h>
#include <vector>
#include <fcntl.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <resource_retriever/retriever.h>
#include <sek_drive/ARMarker.h>
#include <tf/transform_datatypes.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cmath> 
//client includes
#include <netinet/in.h>
#include <resolv.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
////
using namespace std;
//
//marker coordinates relative to camera
double marker_pos_x_ = 0.0;
double marker_pos_y_ = 0.0;
double marker_pos_z_ = 0.0;
double marker_or_x_ = 0.0;
double marker_or_y_ = 0.0;
double marker_or_z_ = 0.0;
double marker_or_w_ = 0.0;
//robot coordinates
double robot_pos_x_ = 0.0;
double robot_pos_y_ = 0.0;
double robot_or_w_ = 0.0;
double robot_or_z_ = 0.0;
//target coordinates
double cof_x_, cof_y_, cof_or_w, cof_or_z;
//turning direction 1 = left, -1 = right, 0 = straight, 2 stopped
int current_direction_ = 3;
int previous_direction_ = 3;
int ORDER_GIVEN = 0;
int ORDER_UNDER_PROCCESS = 0;
int ORDER_COMPLETED = 0;
int ORDER_CANCELLED = 0;
int MARKER_FOUND = 0;
int SEARCHING = 0;
int ALIGNING = 0;
int CUP_FILLED = 0;
int DESTINATION_REACHED = 0;
int RETURNING_ORDER = 0;
int XY_THRES;
int SET_FILL_DESTINATION = 0;
int SET_RETURN_DESTINATION = 0;
int FORW_SPEED = 0;
int TURN_SPEED = 0;
double YAW_THRES;
double RM, LM;

tf::Quaternion target_quat_, robot_quat_, robot_quat_rotated_;
ros::Time time1_, time2_ ;

geometry_msgs::PoseWithCovarianceStamped client_spot;
geometry_msgs::PoseWithCovarianceStamped coffee_spot;
geometry_msgs::PoseWithCovarianceStamped robot_pose;
geometry_msgs::PoseWithCovariance marker_pose;

bool inPosition()
{
    return ((marker_pos_x_ < 0.02) && (marker_pos_x_ > -0.02) && (marker_pos_z_ < 0.40));
}

void markerCallback(const sek_drive::ARMarker::ConstPtr marker_)
{
    //NOTIFIES WHEN THE MARKER IS IN THE CAMERA FRAME
    /*
    marker_pos_x_ = marker_->pose.pose.position.x;
    marker_pos_y_ = marker_->pose.pose.position.y;
    marker_pos_z_ = marker_->pose.pose.position.z;
    marker_or_x_ = marker_->pose.pose.orientation.x;
    marker_or_y_ = marker_->pose.pose.orientation.y;
    marker_or_z_ = marker_->pose.pose.orientation.z;
    marker_or_w_ = marker_->pose.pose.orientation.w;
    */
    //ROS_INFO("IN MARKER CALLBACK");
    marker_pose = marker_->pose;
    if (CUP_FILLED == 0)
    {
        if ((marker_pose.pose.position.z < 2.5) && (marker_pose.pose.position.z > 1.5))
        {
            YAW_THRES = 0.40;
            FORW_SPEED = 180;
            TURN_SPEED = 140;
            //ROS_INFO("MARKER FOUND");
            MARKER_FOUND = 1;
        }
        else if ((marker_pose.pose.position.z < 1.5) && (marker_pose.pose.position.z > 1.0))
        {
            YAW_THRES = 0.15;
            FORW_SPEED = 140;
            TURN_SPEED = 100;
            //ROS_INFO("MARKER FOUND");
            MARKER_FOUND = 1;
        }
        else if (marker_pose.pose.position.z > 0.8)
        {
            YAW_THRES = 0.02;
            FORW_SPEED = 80;
            TURN_SPEED = 80;
            //ROS_INFO("MARKER FOUND");
            MARKER_FOUND = 1;
        }
    }
    //tf::quaternionMsgToTF(marker_->pose.pose.orientation, target_quat_);
    /*
    if (CUP_FILLED == 0 )
    {
        if ((MARKER_FOUND == 0) && (marker_pose.pose.position.z < 1.0))
        {
            ROS_INFO("MARKER FOUND");
            MARKER_FOUND = 1;
        }
    }
    */
    else
    {
        MARKER_FOUND==0;
    }
    time1_ = ros::Time::now();
    return;
}
int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "align_script");
    ros::NodeHandle n;
    ros::Duration(0.1).sleep();
    ros::Subscriber sub_ = n.subscribe("ar_pose_marker", 1, markerCallback);
    ros::Publisher pub1_ = n.advertise<std_msgs::Int32MultiArray>("motor_commands", 50);
    ros::Publisher pub2_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("current_robot_destination", 50);
    geometry_msgs::Quaternion final_quat;
    geometry_msgs::Quaternion temp_quat_;
    double roll, pitch, yaw;
    tf::Quaternion rotation_quat_;
    temp_quat_.x = 0.0;
    temp_quat_.y = 0.0;
    temp_quat_.w = cof_or_w;
    temp_quat_.z = cof_or_z;
    tf::quaternionMsgToTF(temp_quat_, target_quat_);
    temp_quat_.x = 0.0;
    temp_quat_.y = 0.0;
    temp_quat_.w = 1.0;
    temp_quat_.z = 0.0;
    tf::quaternionMsgToTF(temp_quat_, rotation_quat_);
    target_quat_ = target_quat_ * rotation_quat_;
    std_msgs::Int32MultiArray motor_commands;
    motor_commands.data.clear();
    ROS_INFO("STARTING ALIGNMENT");
    cout<<"Starting Alignment"<<endl;
    ros::Duration(0.01).sleep();
    
    //
    //ARMarker Default position is considered face up
    //when mounted on wall 
    //axis z : distance
    //axis x : angle, < 0 left side, > 0 right side
    while (ros::ok())
    {   /*DEBUGGING*/
        ros::spinOnce();
        if (MARKER_FOUND == 1 )
        {   
                if ((ros::Time::now().toSec() - time1_.toSec()) > 0.2) //check if the marker has left our f.o.v.
                {
                    MARKER_FOUND = 0;
                    previous_direction_ = current_direction_;
                    current_direction_ = 2;
                    motor_commands.data.push_back(0);
                    motor_commands.data.push_back(0);
                    pub1_.publish(motor_commands);
                    motor_commands.data.clear();
                    ROS_INFO("MARKER LOST");
                }
                else
                {
                    //if ((marker_pose.pose.position.x < 0.02) && (marker_pose.pose.position.x > -0.02))
                    if ((marker_pose.pose.position.x < YAW_THRES) && (marker_pose.pose.position.x > -YAW_THRES))
                    //if ((marker_pos_x_ < 0.02) && (marker_pos_x_ > -0.02))
                    {
                        if (previous_direction_==3 && current_direction_==3)
                        {
                            ROS_INFO("CENTERED AR MARKER");
                            previous_direction_ = 0 ;
                            current_direction_ = 0;
                            //motor_commands.data.push_back(-160);
                            //motor_commands.data.push_back(160);
                            motor_commands.data.push_back(-FORW_SPEED);
                            motor_commands.data.push_back(FORW_SPEED);
                            ROS_INFO("MOVING FORWARD");
                            pub1_.publish(motor_commands);
                            motor_commands.data.clear();
                        }
                        else
                        {
                            current_direction_ = 0;
                            if ((marker_pose.pose.position.z > 0.40) && (current_direction_ != previous_direction_))
                            //if( (marker_pos_z_ > 0.30) && (current_direction_ != previous_direction_))
                            {
                                previous_direction_ = current_direction_;
                                ROS_INFO("CENTERED AR MARKER");
                                //motor_commands.data.push_back(-160);
                                //motor_commands.data.push_back(160);
                                motor_commands.data.push_back(-FORW_SPEED);
                                motor_commands.data.push_back(FORW_SPEED);
                                ROS_INFO("MOVING FORWARD");
                                pub1_.publish(motor_commands);
                                motor_commands.data.clear();
                            }
                            else if (marker_pose.pose.position.z <= 0.40)
                            {
                                ROS_INFO("IN POSITION");
                                motor_commands.data.push_back(0);
                                motor_commands.data.push_back(0);
                                pub1_.publish(motor_commands);
                                motor_commands.data.clear();
                                ros::Duration(10.0).sleep();
                            }
                        }
                    }
                    //else if (marker_pose.pose.position.x < - 0.02)
                    else if (marker_pose.pose.position.x < - YAW_THRES)
                    //else if (marker_pos_x_ < -0.02) //marker is at the right side of the camera
                    {
                        current_direction_ = -1;
                        if (current_direction_ != previous_direction_)
                        {
                            previous_direction_ = current_direction_ ;
                            //motor_commands.data.push_back(-120);
                            //motor_commands.data.push_back(-120);
                            motor_commands.data.push_back(-TURN_SPEED);
                            motor_commands.data.push_back(-TURN_SPEED);
                            ROS_INFO("ROTATING LEFT TO ALIGN");
                            pub1_.publish(motor_commands);
                            motor_commands.data.clear();
                        }
                    }
                    //else if (marker_pose.pose.position.x > 0.02)
                    else if (marker_pose.pose.position.x > YAW_THRES)
                    //else if (marker_pos_x_ > 0.02) //marker is at the left of the camera
                    {
                        current_direction_ = 1;
                        if (current_direction_ != previous_direction_)
                        {
                            previous_direction_ = current_direction_ ;
                            //current_direction_ = 1;
                            //motor_commands.data.push_back(120);
                            //motor_commands.data.push_back(120);
                            motor_commands.data.push_back(TURN_SPEED);
                            motor_commands.data.push_back(TURN_SPEED);
                            ROS_INFO("ROTATING RIGHT TO ALIGN");
                            pub1_.publish(motor_commands);
                            motor_commands.data.clear();
                        }
                    }
                }
        }
    }
    return 0;
}
