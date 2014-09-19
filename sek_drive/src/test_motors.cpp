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

//XARALAMPOS
#include <std_msgs/Int32MultiArray.h>
std_msgs::Int32MultiArray motor_commands;
//

using namespace std;

#define PI 3.14159265358979323846264338
#define DIAMETER 0.1524 //m,6 INCHES
#define WHEEL_BASE_WIDTH 0.40 //m,  
#define TWOPI /*360*/6.2831853070
#define M_PI 3.14159265358979323846  /* pi */
#define RADS 57.2958
#define MAX_SPEED 1000 //command
#define ACCELERATION 80
/*
  max_vel_x: 0.31416
  #1.9974 
  min_vel x: 0.04
  #NOT REVERSE SPEED MUST BE POSITIVE
  #-2.08721
  max_rotational_vel: 1.5708
  min_in_place_rotational_vel: 1.0
 */
/////////////////
int ESD = 0; //emergency shutdown signal
int REC = 0;
int FIRST_POSE_SET = 0;
int PLAYING = 0;
int CAMERA_ON = 0;
int SCAN_RECORD = 0;
int ALLIGNING = 0;
int ODOMETRY_MODE = 1; //1 for tick count, 2 for RPM count
int PUB_TF = 1;
int PUB_ODOM = 1;
int RECORDING = 1;

double MAX_LIN_VEL = 0.31416;
double MIN_LIN_VEL = 0.04;
double MAX_ROT_VEL = 1.5708;
double MIN_ROT_VEL = 1.0;
double MAX_SPEED_LIMIT;
double RC_MAX_SPEED_LIMIT;

nav_msgs::Path path_to_send;
////////////////////////////

int encoder_ppr = 920;
int encoder_cpr;
int LM = 0;
int RM = 0;
int lenc = 0, renc = 0, lenc_prev = 0, renc_prev = 0, lenc_init = 0, renc_init = 0, enc_errors = 0, lenc2, renc2;
int MXRPM ;
int imu_counter = 0;
double xx = 0, yy = 0, tt = 0;
double imu_angle = 0;

bool firstOdom = true;
bool rp_from_imu = false;
bool y_from_imu = false;

RoboteqDevice device;

ros::Time prev_time, current_time, enc_loop_time, bat_loop_time, tf_loop_time, mot_loop_time, imu_cur_time, imu_prev_time;
ros::Publisher odom_pub;
ros::Publisher encoder_pub;
ros::Publisher encoder_pub_ticks;
ros::Publisher pose_pub;
ros::Publisher pose_pub2;
ros::Subscriber encoder_sub;
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

void queryBat() 
{
    int status = -1;
    int battery_lvl;
    if((status =  device.GetValue(_VOLTS, 2, battery_lvl)) != RQ_SUCCESS)
    {
        ROS_ERROR("Error reading battary voltage");
    }
    else 
    {
        //ROS_INFO("BATTERY LEVEL : %d", battery_lvl/10);
    }
}

void encoderCallback(const sek_drive::encoders::ConstPtr& msg)
{
    if (RECORDING == 1)
    {
        renc = msg->right_wheel;
        lenc = msg->left_wheel;
    }
}

void alignCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
    std::vector<int>::const_iterator it = array->data.begin();
    RM = *it;
    it++;
    LM = *it;
    device.SetCommand(_GO,1, RM);// RIGHT
    device.SetCommand(_GO,2, LM);//LEFT
    cout<<"ALLIGNING WITH AR MARKER"<<endl;
    cout<<"LEFT MOTOR :"<< LM<<endl;
    cout<<"RIGHT MOTOR :"<<RM<<endl;
    return;
}

void esdCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data==1)
    {
        ESD = 1;
    }
    else
    {
        ESD = 0;
    }
}
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    //ROS_INFO("imu yaw:%f",imu_yaw);
    imu_cur_time = ros::Time::now();
    double ang_vel = msg->angular_velocity.z;
    imu_angle = imu_angle + (imu_cur_time - imu_prev_time).toSec() * ang_vel;
    imu_counter ++;
    if (imu_counter == 20)
    {
        imu_counter = 0;
        imu_angle = imu_angle / 20;
        ROS_INFO("%f", imu_angle);
        q_imu = tf::createQuaternionMsgFromRollPitchYaw(0, 0, wrapToPi(imu_angle));
        ROS_INFO("Q_IMU");
        ROS_INFO("%f", q_imu.x);
        ROS_INFO("%f", q_imu.y);
        ROS_INFO("%f", q_imu.z);
        ROS_INFO("%f", q_imu.w);
        ros::Duration(0.3).sleep();
    }
    //imu_yaw=tf::getYaw(msg->orientation);
    //ROS_INFO("imu yaw:%f",tf::getYaw(msg->orientation));
}


void readEnc()
{
    int status;
    
    if (!ros::ok() || !device.IsConnected())
        return;
    ros::Time now = ros::Time::now();
    double delta = (now - prev_time).toSec();
    
    //READ MOTOR POSITION
    if(ODOMETRY_MODE==1) 
    {
        if((status = device.GetValue(_C, 1, renc)) !=RQ_SUCCESS)
        {
            cout<<"failed -->"<<status<<endl;
            ros::Duration(5.0).sleep();
            enc_errors++;
        }
        else
        {
            //cout<<"Right Motor Encoder : "<<renc<<endl;
            //ros::Duration(0.1).sleep();
        }
        if((status = device.GetValue(_C, 2, lenc)) !=RQ_SUCCESS)
        {
            cout<<"failed -->"<<status<<endl;
            ros::Duration(5.0).sleep();
            enc_errors++;
        }
        else
        {
            //cout<<"Left Motor Encoder : "<<lenc<<endl;
            //ros::Duration(0.1).sleep();
        }

        renc=-renc;
        lenc = lenc - lenc_init;
        renc = renc - renc_init;
        sek_drive::encoders encoder_ticks;
        encoder_ticks.header.stamp = now;
        encoder_ticks.header.frame_id = "base_link";
        encoder_ticks.time_delta = delta;
        encoder_ticks.left_wheel = lenc;
        encoder_ticks.right_wheel = -renc;
        encoder_pub_ticks.publish(encoder_ticks);
    }
    else
    {
        if((status = device.GetValue(_ABSPEED, 1, renc2)) !=RQ_SUCCESS)
        {
            cout<<"failed -->"<<status<<endl;
            ros::Duration(5.0).sleep();
            enc_errors++;
        }
        else
        {
            //cout<<"Right Motor Encoder : "<<renc<<endl;
            ros::Duration(0.01).sleep();
        }
        if((status = device.GetValue(_ABSPEED, 2, lenc2)) !=RQ_SUCCESS)
        {
            cout<<"failed -->"<<status<<endl;
            ros::Duration(5.0).sleep();
            enc_errors++;
        }
        else
        {
            //cout<<"Left Motor Encoder : "<<lenc<<endl;
            ros::Duration(0.01).sleep();
        }
        sek_drive::encoders encoder_msg;
        encoder_msg.header.stamp = now;
        encoder_msg.header.frame_id = "base_link";
        encoder_msg.time_delta = delta;
        encoder_msg.left_wheel = lenc2;
        encoder_msg.right_wheel = renc2;
        encoder_pub.publish(encoder_msg);
    }
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{   
    int status;
    /*
    if (ESD = 1)
    {
        if((status = device.SetCommand(_GO, 2,  0) != RQ_SUCCESS))
        {
            cout<<"failed --> \n"<<status<<endl;
        }
        else 
        {
            ROS_INFO("EMERGENCY STOP ENGAGED");
        }
        ros::Duration(0.01).sleep(); 
        if((status = device.SetCommand(_GO, 1,- 0)) != RQ_SUCCESS)
        {
            cout<<"failed --> \n"<<status<<endl;
            device.Disconnect();
            ros::shutdown();
        }
        else
        {
           ROS_INFO("EMERGENCY STOP ENGAGED"); 
        }
        return;
    }
    */
    //SCALE VELOCITIES TO MOTOR COMMANDS
    double L_V = (-1 * (1 - ((msg->linear.x - ((-0.66666666))) / (0.66666666 - ( - 0.66666666))))  +
            ((msg->linear.x - ((-0.66666666))) / (0.66666666 - ( - 0.66666666))))*1000;
    
    double A_V = (-1 * (1 - ((msg->angular.z - ((-1.04000))) / (1.04000 - ( - 1.04000))))  +
            ((msg->angular.z - ((-1.04000))) / (1.04000 - ( - 1.04000))))*1000;
            
    //ROS_INFO("msg.linear_x : %f       msg.anglular.z : %f", msg->linear.x, msg->angular.z);
    //ROS_INFO("L_V : %f            A_V : %f", L_V, A_V);
    /*
    if (abs(L_V) < 200)
    {
        ROS_INFO("IF #1");
        L_V  = 0;
    }
    if (abs(A_V) < 200)
    {
        ROS_INFO("IF #2");
        A_V = 0;
    }
    */
    if (L_V == 0)//akinito oxhma
    {
        //ROS_INFO("LV = 0");
        if (A_V!=0)//epitopia peristrofi
        {   //ROS_INFO("1");
            RM = -A_V;
            LM = -A_V;
        }
        else
        {   //ROS_INFO("STOP");
            RM = 0;
            LM = 0;
        }
    }
    else if ((L_V != 0) || (A_V !=0))
    {
        RM = -L_V;
        LM = L_V;
        if (L_V > 0)//kinisi mprosta
        {   //ROS_INFO("MPROSTA");
            if (A_V > 0)//strofi aristera
            {   //ROS_INFO("ARISTERA");
                RM = RM - A_V;
                if (RM < (-1000))
                {
                    RM = -1000;
                }
                LM = LM + A_V;
                if (LM > 400)
                {
                    LM = 400;
                }
            }
            else if (A_V < 0)//strofi de3ia
            {   //ROS_INFO("DEKSIA");
                LM = LM - A_V;
                if (LM > 1000)
                {
                    LM = 1000;
                }
                RM = RM + A_V;
                if (RM < (-400))
                {
                    RM = -400;
                }
            }
        }
        else if (L_V < 0)//kinisi pisw
        {   //ROS_INFO("PISW");
            RM = -L_V; //>0
            LM = L_V; //<0
            if (A_V > 0)//strofi aristera
            {   //ROS_INFO("ARISTERA");
                RM = RM + A_V;
                if (RM > 1000)
                {
                    RM = 1000;
                }
                LM = LM + A_V;
                if (LM > (-400))
                {
                    LM = -400;
                }
            }
            else if (A_V < 0)//strofi de3ia
            {   //ROS_INFO("DEKSIA");
                LM = LM + A_V;
                if (LM < (-1000))
                {
                    LM = - 1000;
                }
                RM = RM + A_V;
                if (RM < 400)
                {
                    RM = 400;
                }
            }
        }
    }
    //ROS_INFO ("LM : %d        RM : %d",LM, RM);
    if((status = device.SetCommand(_GO,1, RM)) != RQ_SUCCESS)
    {
        cout<<"failed --> "<<status<<endl;
        ros::Duration(5.0).sleep();
        //ros::shutdown();s
    }
    else
    {
        //cout<<"MOTOR 1 SPEED  "<<RM<<endl;
    }
    if((status = device.SetCommand(_GO,2, LM)) !=RQ_SUCCESS)
    {
        cout<<"failed -->"<<status<<endl;
        ros::Duration(5.0).sleep();
        //ros::shutdown();
    }
    else
    {
        //cout<<"MOTOR 2 SPEED : "<<LM<<endl;
    }
    
}

void cmdVelCallback_2(const geometry_msgs::Twist::ConstPtr& msg)
{
    int status;
    double vx, va;
    vx = (-1 * (1 - ((msg->linear.x - ((-MAX_LIN_VEL))) / (MAX_LIN_VEL - ( - MAX_LIN_VEL))))  +
            ((msg->linear.x - ((-MAX_LIN_VEL))) / (MAX_LIN_VEL - ( - MAX_LIN_VEL))))*(MAX_SPEED)*(MAX_SPEED_LIMIT);
    va = (-1 * (1 - ((msg->angular.z - ((-MAX_ROT_VEL))) / (MAX_ROT_VEL - ( - MAX_ROT_VEL))))  +
            ((msg->angular.z - ((-MAX_ROT_VEL))) / (MAX_ROT_VEL - ( - MAX_ROT_VEL))))*(MAX_SPEED)*(MAX_SPEED_LIMIT);
    LM = (vx - va*WHEEL_BASE_WIDTH/2);
    RM = -(vx + va*WHEEL_BASE_WIDTH/2);
    
    ROS_INFO("RM : %d", RM);
    ROS_INFO("LM : %d", LM);
    if((status = device.SetCommand(_GO,1, RM)) != RQ_SUCCESS) //right motor
    {
        cout<<"motor command M1 failed --> "<<status<<endl;
        ros::Duration(5.0).sleep();
    }
    else
    {
        //cout<<"MOTOR 1 SPEED  "<<RM<<endl;
    }
    if((status = device.SetCommand(_GO,2, LM)) != RQ_SUCCESS) //left motor
    {
        cout<<"motor command M1 failed --> "<<status<<endl;
        ros::Duration(5.0).sleep();
    }
}
void motorCommandCallback(const std_msgs::Int32MultiArray::ConstPtr& motor_comms)
{
    int status;
    if((status = device.SetCommand(_GO,1, motor_comms->data[0])) != RQ_SUCCESS) //right motor
    {
        cout<<"motor command M1 failed --> "<<status<<endl;
        ros::Duration(5.0).sleep();
    }
    else
    {
        //cout<<"MOTOR 1 SPEED  "<<RM<<endl;
    }
    if((status = device.SetCommand(_GO,2, motor_comms->data[1])) != RQ_SUCCESS) //left motor
    {
        cout<<"motor command M1 failed --> "<<status<<endl;
        ros::Duration(5.0).sleep();
    }
    else
    {
        //cout<<"MOTOR 1 SPEED  "<<RM<<endl;
    }
}

void teleopCallback2(const sensor_msgs::Joy::ConstPtr& msg)
{
    int status;
    /*
    if (ESD = 1)
    {
        if((status = device.SetCommand(_GO, 2,  0) != RQ_SUCCESS))
        {
            cout<<"failed --> \n"<<status<<endl;
        }
        else 
        {
            ROS_INFO("TCB2a EMERGENCY STOP ENGAGED");
        }
        ros::Duration(0.01).sleep(); 
        if((status = device.SetCommand(_GO, 1,- 0)) != RQ_SUCCESS)
        {
            cout<<"failed --> \n"<<status<<endl;
            device.Disconnect();
            ros::shutdown();
        }
        else
        {
           ROS_INFO("TCB2b EMERGENCY STOP ENGAGED"); 
        }
        return;
    }
    */
    double L_V = msg->axes[1]*1000;
    double A_V = msg->axes[0]*1000;
    //ROS_INFO("L_V : %f        A_V : %f",L_V, A_V);
    if((L_V != 0) || (A_V != 0))
    {
        if(L_V == 0)//akinito oxhma
        {
            if(A_V!=0)//epitopia peristrofi
            {   //ROS_INFO("1");
                RM = -A_V;
                LM = -A_V;
            }
            else
            {   //ROS_INFO("STOP");
                RM = 0;
                LM = 0;
            }
        }
        else if ((L_V != 0) || (A_V !=0))
        {
            RM = -L_V;
            LM = L_V;
            if(L_V > 0)//kinisi mprosta
            {   //ROS_INFO("MPROSTA");
                if(A_V > 0)//strofi aristera
                {   //ROS_INFO("ARISTERA");
                    RM = RM - A_V;
                    if (RM < (-1000))
                    {
                        RM = -1000;
                    }
                    LM = LM + A_V;
                    if (LM > 400)
                    {
                        LM = 400;
                    }
                }
                else if (A_V < 0)//strofi de3ia
                {   //ROS_INFO("DEKSIA");
                    LM = LM - A_V;
                    if (LM > 1000)
                    {
                        LM = 1000;
                    }
                    RM = RM + A_V;
                    if (RM < (-400))
                    {
                        RM = -400;
                    }
                }
            }
            else if (msg->axes[1]<0)//kinisi pisw
            {   //ROS_INFO("PISW");
                RM = -L_V; //>0
                LM = L_V; //<0
                if(A_V > 0)//strofi aristera
                {   //ROS_INFO("ARISTERA");
                    RM = RM + A_V;
                    if (RM > 1000)
                    {
                        RM = 1000;
                    }
                    LM = LM + A_V;
                    if (LM > (-400))
                    {
                        LM = -400;
                    }
                }
                else if (A_V < 0)//strofi de3ia
                {   //ROS_INFO("DEKSIA");
                    LM = LM + A_V;
                    if (LM < (-1000))
                    {
                        LM = - 1000;
                    }
                    RM = RM + A_V;
                    if (RM < 400)
                    {
                        RM = 400;
                    }
                }
            }
        }
        //ROS_INFO ("LM : %d        RM : %d",LM, RM);
        /*
         * LIMIT ROBOT VELOCITY AT 75% OF MAXIMUM FOR SAFETY AND 
         * BETTER CONTROL OVER ACCELERATION/DECCELERATION 
         * CHANGES MUST BE ALSO MADE IN THE AMCL CONFIG FILES
         * FOR THE NAVIGATION PACKAGE
        */
        if(abs(LM) > (MAX_SPEED*MAX_SPEED_LIMIT))
        {
            LM = (LM/abs(LM)) * MAX_SPEED * RC_MAX_SPEED_LIMIT;
        }
        if(abs(RM) > (MAX_SPEED*MAX_SPEED_LIMIT))
        {
            RM = (RM/abs(RM)) * MAX_SPEED * RC_MAX_SPEED_LIMIT;
        }
        if((status = device.SetCommand(_GO,1, RM)) != RQ_SUCCESS)
        {
            cout<<"motor command M1 failed --> "<<status<<endl;
            ros::Duration(5.0).sleep();
        }
        else
        {
            //cout<<"MOTOR 1 SPEED  "<<RM<<endl;
        }
        if((status = device.SetCommand(_GO,2, LM)) !=RQ_SUCCESS)
        {
            cout<<"motor command M2 failed -->"<<status<<endl;
            ros::Duration(5.0).sleep();
        }
        else
        {
            //cout<<"MOTOR 2 SPEED : "<<LM<<endl;
        }
    }
    else 
    {
        if((status = device.SetCommand(_GO, 1, 0)) != RQ_SUCCESS)
        {
            cout<<"Stopping M1 failed --> "<<status<<endl;
            ros::Duration(5.0).sleep();
        }
        else
        {
            //cout<<"MOTOR 1 SPEED  "<<RM<<endl;
        }
        if((status = device.SetCommand(_GO, 2, 0)) !=RQ_SUCCESS)
        {
            cout<<"Stopping M2 failed -->"<<status<<endl;
            ros::Duration(5.0).sleep();
        }
        else
        {
            //cout<<"MOTOR 2 SPEED : "<<LM<<endl;
        }
        if(msg->buttons[1]==1)//CIRCLE BUTTON
        {
            //ROS_INFO("HEHEHEHEHEHEHEHEHHEHE");
            system("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID '{}' &");
        }
        if(msg->buttons[3]==1)//SQUARE BUTTON
        {
            ROS_INFO("SAVING MAP as \"mymap\"");
            system("rosrun map_server map_saver -f mymap &");
        }
        if(msg->buttons[2]==1)//Χ ΒUTTON
        {   
            if(CAMERA_ON==0)
            {
                cout<<"Camera on"<<endl;
                ROS_INFO("Starting Streaming");
                system("roslaunch sek_drive mast_kinect.launch &");
                CAMERA_ON = 1 ;
            }
            else
            {
                cout<<"Camera off"<<endl;
                ROS_INFO("Shutting Down Streaming");
                system("rosnode kill /camera/camera_nodelet_manager &");
                system("rosnode kill /mjepg_server &");
                ros::Duration(3).sleep();
                CAMERA_ON = 0;
                //system("rosnode cleanup")
                system("/home/skel/cleanup.sh ");
            }
        }
        if(msg->buttons[6]==1)//START RECORDING //L1
        {
            
            if(REC==0)
            {
                REC=1; 
                ROS_INFO("Start Recording");
                //system("roslaunch sek_drive record.launch &");
                //ros::Duration(3).sleep();
            }
            
        }
        if(msg->buttons[7]==1)//STOP RECORDING //R1
        {
            if(REC==1)
            {
                REC=0;
                ROS_INFO("Stop Recording");
                //pose_vector_to_use =  pose_vector;
                //pose_vector.clear();
                FIRST_POSE_SET = 0;
                //system("rosnode kill /maneuvers/record_man &");
                //ros::Duration(3).sleep();
                //system("/home/skel/cleanup.sh &");
            } 
        }
        if(msg->buttons[4]==1)//START PLAYING //L2
        {
            if((PLAYING==0)&&(REC==0))
            {
                PLAYING=1;
                geometry_msgs::PoseStamped pose_to_add;
                ROS_INFO("Playing Maneuver");
                //system("rosbag play /home/skel/.ros/maneuver.bag &");
                /*
                for(unsigned i = 0; i < pose_vector.size(); i++)
                {
                    pose_to_add.pose.position.x = pose_vector[i].x;
                    pose_to_add.pose.position.y = pose_vector[i].y;
                    pose_to_add.pose.orientation.w = pose_vector[i].w;
                    path_to_send.poses.push_back(pose_to_add);
                }
                */
            }
        }
        if(msg->buttons[5]==1)//STOP PLAYING //R2
        {
            if(PLAYING==1)
            {
                PLAYING=0;
                ROS_INFO("Stopping Maneuver");
                //system("killall -9 play");
                //system("/home/skel/cleanup.sh &");
                
            }
        }
        if((msg->buttons[10]==1)&&(msg->buttons[11]==1))
        {   
            ROS_INFO("Restarting Python Server");
            system("rosrun robot_server robotserverBl.py &");
        }
        else if (msg->buttons[10]==1)
        {
            
        }
        else if (msg->buttons[11]==1)
        {

        }
        else if((msg->buttons[8]==1)&&(msg->buttons[9]==1))//SELECT & START
        {
            printf("Emergency Shutdown\n");	
            if((status = device.SetCommand(_GO, 2,  0) != RQ_SUCCESS))
            {
                cout<<"failed --> \n"<<status<<endl;
            }
            else 
            {
            }
            ros::Duration(0.01).sleep(); 
            if((status = device.SetCommand(_GO, 1,- 0)) != RQ_SUCCESS)
            {
                cout<<"failed --> \n"<<status<<endl;
                device.Disconnect();
                system("/home/skel/cleanup.sh &");
                ros::shutdown();
            }
            else
            {
            }
            ROS_INFO("SHUTTING DOWN");
            ros::shutdown();
        }
        if((msg->buttons[9]==1)&&(msg->buttons[8]==0))//START
        {
            ROS_INFO("Giving Start Signal");
            //system("/home/skel/start");
            //return;
        }
    }
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
        
        double l_w = diff_lenc * 2.0 * PI / (double)encoder_cpr / delta_time;
        double r_w = diff_renc * 2.0 * PI / (double)encoder_cpr / delta_time;
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
    {
        double l_v = (2*M_PI*(DIAMETER/2.0)*lenc2)/60;
        double r_v = (2*M_PI*(DIAMETER/2.0)*renc2)/60;
        //ROS_INFO("L_V : %f     R_V : %f",l_v ,r_v);
        double v = (l_v + r_v) / 2.0;
        //ROS_INFO("V AFTER : %f",v);
        double w = (r_v - l_v) / WHEEL_BASE_WIDTH;
        
        tt = tt + w * delta_time;
        tt = wrapToPi(tt);
        xx = xx + (v * cos(tt)) * delta_time ;
        yy = yy + (v * sin(tt)) * delta_time;
        
        //ROS_INFO("TT BEFORE WRAP : %f",tt);
        
        quat = tf::createQuaternionMsgFromRollPitchYaw(wrapToPi(roll), wrapToPi(pitch), tt);

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
    
    n.getParam("test_motors/odom_mode", ODOMETRY_MODE);
    n.getParam("test_motors/pub_tf", PUB_TF);
    n.getParam("test_motors/pub_odom", PUB_ODOM);
    n.getParam("test_motors/max_speed_lim", MAX_SPEED_LIMIT);
    n.getParam("test_motors/rc_max_speed_lim", RC_MAX_SPEED_LIMIT);
    
    ROS_INFO("Odometry Mode : %d", ODOMETRY_MODE);
    ROS_INFO("Publish TF : %d", PUB_TF);
    ROS_INFO("Publish Odometry Messages : %d", PUB_ODOM);
    ROS_INFO("Max Speed %% Limit : %f", MAX_SPEED_LIMIT);
    prev_time = ros::Time::now();
    
    imu_prev_time = ros::Time::now();
    
    ros::Duration(0.1).sleep();
    odom_broadcaster = new tf::TransformBroadcaster;
    odom_pub = n.advertise<nav_msgs::Odometry > ("/odom", 10);
    encoder_pub = n.advertise<sek_drive::encoders>("/encoders", 10);
    encoder_pub_ticks = n.advertise<sek_drive::encoders>("/encoder_ticks", 10);
    pose_pub2 = n.advertise<geometry_msgs::PoseStamped>("/poseStamped",5);
    
    ros::Subscriber sub = n.subscribe("/joy", 1, teleopCallback2);
    ros::Subscriber imu_sub = n.subscribe("/imu/data", 10, imuCallback);
    ros::Subscriber align_sub = n.subscribe("/motor_commands", 1, alignCallback);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 100, cmdVelCallback_2);
    
    //ROS_INFO("- SetConfig(_DINA, 1, 1)...");
    if((status = device.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      //ROS_INFO("succeeded.");
      
    ros::Duration(0.01).sleep(); //sleep for 10 ms

    int result;
    int MAX_RPM;
    enc_loop_time = ros::Time::now();
    
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
        current_time = ros::Time::now();
        if ((current_time.toSec() - enc_loop_time.toSec())>=0.2)
        {
            readEnc();
            calcOdom();
            //pub2.publish(motor_commands);
            enc_loop_time = current_time;
        }
    }
    device.Disconnect();
    return 0;
}
