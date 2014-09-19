#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sek_drive/encoders.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"

#include <tf/transform_broadcaster.h>
#include "robodev.cpp"
#include <sdc2130_skel/RoboteqDevice.h>
#include <sdc2130_skel/ErrorCodes.h>
#include <sdc2130_skel/Constants.h>
#include "ros/ros.h"
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cmath>

using namespace std;

#define DIAMETER 0.1524 //m,6 INCHES
#define WHEEL_BASE_WIDTH 0.43 //m,
#define M_PI 3.14159265358979323846  /* pi */
#define MAX_SPEED 1000 //command
#define MAX_LIN_VEL  2
#define MIN_LIN_VEL  -2
#define MAX_ROT_VEL  4
#define MIN_ROT_VEL  -4
#define ACCELERATION 80

int ODOMETRY_MODE = 1; //1 for tick count, 2 for RPM count
int PUB_TF = 1;
int PUB_ODOM = 1;

double MAX_SPEED_LIMIT;

nav_msgs::Path path_to_send;
////////////////////////////

int encoder_ppr = 450;
int encoder_cpr;
int LM = 0;
int RM = 0;
int lenc = 0, renc = 0, lenc_prev = 0, renc_prev = 0, lenc_init = 0, renc_init = 0, enc_errors = 0, lenc_2, renc_2;
int MXRPM ;
int imu_counter = 0;
double xx = 0, yy = 0, tt = 0;
double imu_angle = 0;

bool firstOdom = true;
bool rp_from_imu = false;
bool y_from_imu = false;

RoboteqDevice device;

ros::Time prev_time, current_time, enc_loop_time, enc_loop_time_2, bat_loop_time, tf_loop_time, mot_loop_time, imu_cur_time, imu_prev_time;
ros::Publisher odom_pub;
ros::Publisher encoder_pub;
ros::Publisher pose_pub;
ros::Publisher pose_pub2;
ros::Subscriber encoder_sub;
ros::Subscriber encoder_tick_sub;
tf::TransformBroadcaster *odom_broadcaster;
geometry_msgs::Quaternion q_imu = tf::createQuaternionMsgFromYaw(0);

double wrapToPi(double angle)
{
    angle += M_PI;
    bool is_neg = (angle < 0);
    angle = fmod(angle, (2.0 * M_PI));
    if (is_neg) 
    {
        angle += (2.0 * M_PI);
    }
    angle -= M_PI;
    return angle;
}

void calcOdom()
{
    //cout<<"Odometry Callback"<<endl;
    ros::Time now = ros::Time::now();
    double delta_time = (now.toSec() - prev_time.toSec());
    //ROS_INFO("Delta Time : %f",delta_time);
    prev_time = now;
    
    geometry_msgs::Quaternion quat;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    tf::Quaternion q;
    tf::quaternionMsgToTF(q_imu, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    if (firstOdom)
    {
        renc_prev = renc;
        lenc_prev = lenc;
        xx = 0;
        yy = 0;
        tt = 0;
        firstOdom = false;
    }
    
    if (ODOMETRY_MODE == 1)
    {
        int diff_lenc = lenc - lenc_prev;
        int diff_renc = renc - renc_prev;
        
        renc_prev = renc;
        lenc_prev = lenc;
        //ROS_INFO("diff_lenc : %d     diff_renc : %d",diff_lenc ,diff_renc);
        double l_w = diff_lenc * 2.0 * M_PI / (double)encoder_cpr / delta_time;
        double r_w = diff_renc * 2.0 * M_PI / (double)encoder_cpr / delta_time;
        //ROS_INFO("L_W : %f     R_W : %f",l_w ,r_w);
        
        double l_v = l_w * DIAMETER / 2.0;
        double r_v = r_w * DIAMETER / 2.0;
        //ROS_INFO("L_V : %f     R_V : %f",l_v ,r_v);
        double v = (l_v + r_v) / 2.0;
        //ROS_INFO("V AFTER : %f",v);
        double w = (r_v - l_v) / WHEEL_BASE_WIDTH;
        
        if (firstOdom)
        {
            xx = 0;
            yy = 0;
            tt = 0;
            firstOdom = false;
        }
        tt = tt + w * delta_time;
        //ROS_INFO("TT BEFORE WRAP : %f",tt);
        tt = wrapToPi(tt);
        //ROS_INFO("TT : %f", tt);
        //ROS_INFO("V BEFORE CREATE QUATERNION : %f",v);
        //ROS_INFO("TT AFTER WRAP : %f",tt);
        quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, tt);
        
        //ROS_INFO(" sdfgasdgasdga %f",v*cos(tt));
        xx = xx + (v * cos(tt)) * delta_time ;
        yy = yy + (v * sin(tt)) * delta_time;
        //ROS_INFO("%f   %f",xx ,yy);
        //ROS_INFO("%f    %f",v ,w);
        //ROS_INFO("---------");
        if(PUB_ODOM == 1)
        {
            nav_msgs::Odometry odom_msg;
            //odom_msg.header.stamp = now;
            odom_msg.header.frame_id = "odom";
            odom_msg.header.stamp = now;
            odom_msg.pose.pose.position.x = xx;
            //ROS_INFO("%f",xx);
            odom_msg.pose.pose.position.y = yy;
            //ROS_INFO("%f",yy);
            odom_msg.pose.pose.position.z = 0;
            odom_msg.pose.pose.orientation = quat;
            odom_msg.child_frame_id = "base_link";
            odom_msg.twist.twist.linear.x = v;
            odom_msg.twist.twist.linear.y = 0;
            odom_msg.twist.twist.angular.z = w;
            
            odom_msg.pose.covariance[0] = 0.26298481867953083;
            odom_msg.pose.covariance[1] = -0.007138441796595563;
            odom_msg.pose.covariance[6] = -0.007138441796595563;
            odom_msg.pose.covariance[7] = 0.25096033123823897;
            odom_msg.pose.covariance[14] = 1e100;
            odom_msg.pose.covariance[21] = 1e100;
            odom_msg.pose.covariance[28] = 1e100;
            odom_msg.pose.covariance[35] = 0.06684735983012981;
            odom_msg.twist.covariance = odom_msg.pose.covariance;

            odom_pub.publish(odom_msg);
        }
    }
    else
    {   ROS_INFO("LENC : %d    RENC : %d" ,lenc_2, renc_2);
        double l_v = (2*M_PI*(DIAMETER/2.0)*lenc_2)/60;
        double r_v = (2*M_PI*(DIAMETER/2.0)*renc_2)/60;
        ROS_INFO("L_V : %f     R_V : %f",l_v ,r_v);
        double v = (l_v + r_v) / 2.0;
        ROS_INFO("V AFTER : %f",v);
        ROS_INFO("SUBBED :%f", (r_v-l_v));
        double w = (r_v - l_v) / WHEEL_BASE_WIDTH;
        //w = (2*w)/DIAMETER;
        tt = tt + w * delta_time;
        //ROS_INFO("W : %f", w);
        //ROS_INFO("delta time :%f", delta_time);
        //ROS_INFO("TT BEFORE WRAP : %f",tt);
        tt = wrapToPi(tt);
        xx = xx + (v * cos(tt)) * delta_time ;
        yy = yy + (v * sin(tt)) * delta_time;
        
        ROS_INFO("TT AFTER WRAP : %f", tt);
        ROS_INFO("XX AFTER WRAP : %f", xx);
        ROS_INFO("YY AFTER WRAP : %f", yy);
        //quat = tf::createQuaternionMsgFromRollPitchYaw(wrapToPi(roll), wrapToPi(pitch), tt);
        quat = tf::createQuaternionMsgFromRollPitchYaw(0 , 0 , tt);
        
        nav_msgs::Odometry odom_msg;
        //odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.header.stamp = now;
        odom_msg.pose.pose.position.x = xx;
        //ROS_INFO("%f",xx);
        odom_msg.pose.pose.position.y = yy;
        //ROS_INFO("%f",yy);
        odom_msg.pose.pose.position.z = 0;
        odom_msg.pose.pose.orientation = quat;
        odom_msg.child_frame_id = "base_link";
        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.linear.y = 0;
        odom_msg.twist.twist.angular.z = w;
        /*
        odom_msg.pose.covariance[0] = 0.26298481867953083;
        odom_msg.pose.covariance[1] = -0.007138441796595563;
        odom_msg.pose.covariance[6] = -0.007138441796595563;
        odom_msg.pose.covariance[7] = 0.25096033123823897;
        odom_msg.pose.covariance[14] = 1e100;
        odom_msg.pose.covariance[21] = 1e100;
        odom_msg.pose.covariance[28] = 1e100;
        odom_msg.pose.covariance[35] = 0.06684735983012981;
        odom_msg.twist.covariance = odom_msg.pose.covariance;
        */
        odom_pub.publish(odom_msg);
    }
    

    
    if (PUB_TF == 1)
    {
    //Add TF broadcaster
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = now;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = xx;
        odom_trans.transform.translation.y = yy;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = quat;
        odom_broadcaster->sendTransform(odom_trans);
    }
}

void encoderCallback(const sek_drive::encoders::ConstPtr& msg)
{
    renc_2 = - msg->right_wheel;
    lenc_2 = msg->left_wheel;
    current_time = ros::Time::now();
    if ((current_time.toSec() - enc_loop_time_2.toSec())>=0.05)
    {
        calcOdom();
        enc_loop_time_2 = current_time;
    }
}

void encoderTickCallback(const sek_drive::encoders::ConstPtr& msg)
{
    renc = - msg->right_wheel;
    lenc = msg->left_wheel;
    current_time = ros::Time::now();
    if ((current_time.toSec() - enc_loop_time.toSec())>=0.05)
    {
        calcOdom();
        enc_loop_time_2 = current_time;
    }
}

int main(int argc, char *argv[])
{   
    int status = device.Connect("/dev/ttyACM1");
    if(status != RQ_SUCCESS)
    {
        cout<<"Error connecting to device: "<<status<<"."<<endl;
        return 1;
    }

    device.SetConfig(_RWD, 1, -1);
    device.SetConfig(_RWD, 2, -1);
    
    ros::init(argc, argv, "move_script");
    ros::NodeHandle n;
    
    n.getParam("odom_from_enc/odom_mode", ODOMETRY_MODE);
    n.getParam("odom_from_enc/pub_tf", PUB_TF);
    n.getParam("odom_from_enc/pub_odom", PUB_ODOM);
    n.getParam("odom_from_enc/max_speed_lim", MAX_SPEED_LIMIT);
    
    ROS_INFO("Odometry Mode : %d", ODOMETRY_MODE);
    ROS_INFO("Publish TF : %d", PUB_TF);
    ROS_INFO("Publish Odometry Messages : %d", PUB_ODOM);
    ROS_INFO("Max Speed %% Limit : %f", MAX_SPEED_LIMIT);
    prev_time = ros::Time::now();
    
    imu_prev_time = ros::Time::now();
    
    ros::Duration(0.1).sleep();
    odom_broadcaster = new tf::TransformBroadcaster;
    odom_pub = n.advertise<nav_msgs::Odometry > ("/odom", 10);
    pose_pub2 = n.advertise<geometry_msgs::PoseStamped>("/poseStamped",5);
    ros::Subscriber enc_sub = n.subscribe("/encoders", 100, encoderCallback);
    ros::Subscriber encoder_tick_sub = n.subscribe("/encoder_ticks", 100, encoderTickCallback);
    //ROS_INFO("- SetConfig(_DINA, 1, 1)...");
    if((status = device.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      //ROS_INFO("succeeded.");
      
    ros::Duration(0.01).sleep(); //sleep for 10 ms

    int result;
    int MAX_RPM;
    enc_loop_time = ros::Time::now();
    enc_loop_time_2 = ros::Time::now();
    //ROS_INFO("- GetConfig(_DINA, 1)...");
    if((status = device.GetConfig(_DINA, 1, result)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"returned --> "<<result<<endl;
    
    //ROS_INFO("Reading RPM");
    if((status = device.GetConfig(_MXRPM, 1, MXRPM)) !=RQ_SUCCESS)
        cout<<"reading rpm M1 failed -->"<<status<<endl;
    else
        cout<<"MOTOR 1 MAXRPM : "<<MXRPM<<endl;
        
    if((status = device.GetConfig(_MXRPM, 2, MXRPM)) !=RQ_SUCCESS)
        cout<<"reading rpm M2 failed -->"<<status<<endl;
    else
        cout<<"MOTOR 2 MAXRPM : "<<MXRPM<<endl;
    
    encoder_cpr = encoder_ppr*4;
    //SET ACCELERATION
    /* 
    //ROS_INFO("SETTING ACCELERATION");
    if((status = device.SetConfig(_AC, 1, ACCELERATION)) !=RQ_SUCCESS)
        cout<<"failed -->"<<result<<endl;
    else
        cout<<"MOTOR 1 ACCEL : "<<ACCELERATION<<endl;
    
    if((status = device.SetConfig(_AC, 2, ACCELERATION)) !=RQ_SUCCESS)
        cout<<"failed -->"<<result<<endl;
    else
        cout<<"MOTOR 2 ACCEL : "<<ACCELERATION<<endl;
    
    
    //ROS_INFO("Roboteq -- Successfull setup of the Roboteq SDC2130");
    */
    printf ("Sek Operational\n\n");
    ros::Duration(0.01).sleep(); //sleep for 10 ms
    while (ros::ok())
    {
        ros::spinOnce();
        //current_time = ros::Time::now();
        //if ((current_time.toSec() - enc_loop_time.toSec())>=0.2)
        //{
        //    readEnc();
        //    calcOdom();
        //    enc_loop_time = current_time;
        //}
    }
    device.Disconnect();
    return 0;
}
