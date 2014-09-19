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
#include <sensor_msgs/LaserScan.h> //DELETE

#define PI 3.14159265358979323846264338
#define DIAMETER 0.1524 //m,6 INCHES
#define WHEEL_BASE_WIDTH 0.40 //m,  
#define TWOPI /*360*/6.2831853070
#define M_PI 3.14159265358979323846  /* pi */
#define RADS 57.2958
#define MAX_SPEED 1000 //command
#define ACCELERATION 80

using namespace std;


class sek_controller
{
    
int SCAN_RECEIVED;//DELETE
sensor_msgs::LaserScan gl_scan;//DELETE
bool bullshit; //DELETE
    protected :
        ros::NodeHandle n_;
        int ODOMETRY_MODE;
        int PUB_TF;
        int PUB_ODOM;
        int CAMERA_ON;
        
        double MAX_SPEED_LIMIT;
        double RC_MAX_SPEED_LIMIT;
        
        double max_vel_x;
        double min_vel_x;
        double max_rotational_vel;
        double acc_lim_th;
        double acc_lim_x;
        double acc_lim_y;
        
        int encoder_ppr;
        int encoder_cpr;
        int LM;
        int RM;
        int lenc, renc, lenc_prev, renc_prev, lenc_init, renc_init, enc_errors, lenc2, renc2;
        int MAXRPM;
        double xx, yy, tt;
        
        bool firstOdom;
        RoboteqDevice device;
        
        ros::Time prev_time, current_time, enc_loop_time;
        ros::Publisher odom_pub, encoder_pub, encoder_pub_ticks, pose_pub, pose_pub2;
        ros::Subscriber encoder_sub;
        
        ros::Subscriber hok_in;//DELETE
        tf::TransformBroadcaster *odom_broadcaster;
        //geometry_msgs::Quaternion q_imu;
        
    public:
        sek_controller (ros::NodeHandle& n): n_(n), ODOMETRY_MODE(1), PUB_TF(1), PUB_ODOM(1), CAMERA_ON(0), max_vel_x(0.6), min_vel_x(0.04),
            max_rotational_vel(2.3771), acc_lim_th(0), acc_lim_x(0), acc_lim_y(0), MAX_SPEED_LIMIT(1), RC_MAX_SPEED_LIMIT(0), encoder_ppr(450), encoder_cpr(0), LM(0), RM(0),
            lenc(0), renc(0), lenc_prev(0), renc_prev(0), lenc_init(0), renc_init(0), enc_errors(0), lenc2(0), renc2(0), bullshit(false)/*DELETE*/
            {}
            
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
        
        void calcVelocities()
        {
            
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

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
        {
            int status;
            double vx, va;
            vx = (-1 * (1 - ((msg->linear.x - ((-max_vel_x))) / (max_vel_x - ( - max_vel_x))))  +
                ((msg->linear.x - ((-max_vel_x))) / (max_vel_x - ( - max_vel_x))))*(MAX_SPEED)*(MAX_SPEED_LIMIT);
                    
            va = (-1 * (1 - ((msg->angular.z - ((-max_rotational_vel))) / (max_rotational_vel - ( - max_rotational_vel))))  +
                ((msg->angular.z - ((-max_rotational_vel))) / (max_rotational_vel - ( - max_rotational_vel))))*(MAX_SPEED)*(MAX_SPEED_LIMIT);
             
            vx = ((MAX_SPEED + MAX_SPEED) * (msg->linear.x + max_vel_x)) / (2*max_vel_x) - MAX_SPEED;    
            va = ((MAX_SPEED + MAX_SPEED) * (msg->angular.z + max_rotational_vel)) / (2*max_rotational_vel)- MAX_SPEED;
                    
            ROS_INFO("VX : %f", vx);
            ROS_INFO("VA : %f", va);
            LM = (vx - va);//*(WHEEL_BASE_WIDTH/2));
            RM = -(vx + va);//*(WHEEL_BASE_WIDTH/2));
            /*
            LM = (msg->linear.x - msg->angular.z*WHEEL_BASE_WIDTH/2);
            RM = (msg->linear.x + msg->angular.z*WHEEL_BASE_WIDTH/2);
            ROS_WARN("RM : %d", RM);
            ROS_WARN("LM : %d", LM);
            double v_max = 0.6;
            double v_min = 0;
            double motor_min = 0;
            double motor_max = 500;
            LM = ((motor_max - motor_min) * (LM - v_min)) / (v_max - v_min);
            RM = ((motor_max - motor_min) * (RM - v_min)) / (v_max - v_min);*/
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
        
        void teleopCallback(const sensor_msgs::Joy::ConstPtr& msg)
        {
            int status;
            
            double L_V = msg->axes[3]*1000;
            double A_V = msg->axes[4]*1000;
            
            ROS_INFO("L_V : %f        A_V : %f",L_V, A_V);
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
                /*
                if(abs(LM) > (MAX_SPEED*RC_MAX_SPEED_LIMIT))
                {
					//ROS_INFO("HERE");
                    LM = (LM/abs(LM)) * MAX_SPEED * RC_MAX_SPEED_LIMIT;
                }
                if(abs(RM) > (MAX_SPEED*RC_MAX_SPEED_LIMIT))
                {
					//ROS_INFO("HERE-2");
                    RM = (RM/abs(RM)) * MAX_SPEED * RC_MAX_SPEED_LIMIT;
                }
                */
                if((status = device.SetCommand(_GO,1, RM)) != RQ_SUCCESS)
                {
                    cout<<"motor command M1 failed --> "<<status<<endl;
                    ros::Duration(5.0).sleep();
                }
                else
                {
                    cout<<"MOTOR 1 SPEED  "<<RM<<endl;
                }
                if((status = device.SetCommand(_GO,2, LM)) !=RQ_SUCCESS)
                {
                    cout<<"motor command M2 failed -->"<<status<<endl;
                    ros::Duration(5.0).sleep();
                }
                else
                {
                    cout<<"MOTOR 2 SPEED : "<<LM<<endl;
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
                    //ROS_INFO("CANCELLING GOAL");
                    //system("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID '{}' &");
                }
                if(msg->buttons[3]==1)//SQUARE BUTTON
                {
                    //ROS_INFO("SAVING MAP as \"mymap\"");
                    bullshit = !bullshit;//DELETE
                    if(bullshit){
                        ROS_INFO("TRUE");
                    }
                    else{
                        ROS_WARN("FALSE");
                    }
                    //system("rosrun map_server map_saver -f mymap &");
                }
                if(msg->buttons[2]==1)//Χ ΒUTTON
                {   
                    if(CAMERA_ON==0)
                    {
                        //cout<<"Camera on"<<endl;
                        //ROS_INFO("Starting Streaming");
                        //system("roslaunch sek_drive mast_kinect.launch &");
                        CAMERA_ON = 1 ;
                    }
                    else
                    {
                        //cout<<"Camera off"<<endl;
                        //ROS_INFO("Shutting Down Streaming");
                        //system("rosnode kill /camera/camera_nodelet_manager &");
                        //system("rosnode kill /mjepg_server &");
                        ros::Duration(3).sleep();
                        CAMERA_ON = 0;
                        //system("rosnode cleanup")
                        //system("/home/skel/cleanup.sh ");
                    }
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
                        //system("/home/skel/cleanup.sh &");
                        ros::shutdown();
                    }
                    else
                    {
                    }
                    device.SetCommand(_GO, 1,- 0);
                    device.SetCommand(_GO, 2,- 0);
                    ROS_INFO("SHUTTING DOWN");
                    device.Disconnect();

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

        void readEnc()
        {
            int status;
            
            if (!ros::ok() || !device.IsConnected())
                return;
            ros::Time now = ros::Time::now();
            double delta = (now - prev_time).toSec();
            
            //READ MOTOR POSITION
            /*
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
            */
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
            //tf::Quaternion q;
            //tf::quaternionMsgToTF(q_imu, q);
            //tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            
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
            

            /*
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
            }*/
        }
        
        void calculateVel()
        {
            max_rotational_vel = max_vel_x/(WHEEL_BASE_WIDTH/2);
        }
        int spin()
        {   
            int status = device.Connect("/dev/ttyACM0");
            if(status != RQ_SUCCESS)
            {
                cout<<"Error connecting to device: "<<status<<"."<<endl;
                return 1;
            }

            device.SetConfig(_RWD, 1, -1);
            device.SetConfig(_RWD, 2, -1);
    
            n_.getParam("sek_controller/odom_mode", ODOMETRY_MODE);
            n_.getParam("sek_controller/pub_tf", PUB_TF);
            n_.getParam("sek_controller/pub_odom", PUB_ODOM);
            n_.getParam("sek_controller/max_speed_lim", MAX_SPEED_LIMIT);
            n_.getParam("sek_controller/rc_max_speed_lim", RC_MAX_SPEED_LIMIT);
            n_.getParam("sek_controller/max_vel_x", max_vel_x);
            n_.getParam("sek_controller/min_vel_x", min_vel_x);
            calculateVel();
            n_.getParam("sek_controller/max_rotational_value", max_rotational_vel);
            //
            
            ROS_INFO("Odometry Mode : %d", ODOMETRY_MODE);
            ROS_INFO("Publish TF : %d", PUB_TF);
            ROS_INFO("Publish Odometry Messages : %d", PUB_ODOM);
            ROS_INFO("Max Speed %% Limit : %f", MAX_SPEED_LIMIT);
            ROS_INFO("Max Linear Speed :%f", max_vel_x);
            ROS_INFO("Min Linear Speed :%f", min_vel_x);
            ROS_INFO("Max Rotational Speed :%f", max_rotational_vel);
            prev_time = ros::Time::now();
    
            ros::Duration(0.1).sleep();
            odom_broadcaster = new tf::TransformBroadcaster;
            odom_pub = n_.advertise<nav_msgs::Odometry > ("/odom", 10);
            encoder_pub = n_.advertise<sek_drive::encoders>("/encoders", 10);
            encoder_pub_ticks = n_.advertise<sek_drive::encoders>("/encoder_ticks", 10);
            pose_pub2 = n_.advertise<geometry_msgs::PoseStamped>("/poseStamped",5);
    
            ros::Subscriber sub = n_.subscribe("/joy", 1, &sek_controller::teleopCallback, this);
            
            
            hok_in = n_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &sek_controller::scanCallback, this);//DELETE
            
            ros::Subscriber align_sub = n_.subscribe("/motor_commands", 1, &sek_controller::alignCallback, this);
            ros::Subscriber cmd_vel_sub = n_.subscribe("/cmd_vel", 100, &sek_controller::cmdVelCallback, this);
    
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
            int MXRPM;
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
            
            
            
            
            
            
        //SKATA    
    SCAN_RECEIVED = 0;
    
	float min_angle = gl_scan.angle_min;//save the minimum angle in degrees
	float max_angle = gl_scan.angle_max;//save the maximum angle in degrees
            //SKATA
            
            
            
            
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
                    
                    
                    
                //BULLSHIT STARTS HERE  
                //ROS_INFO("Looping");
                
                if(bullshit){
                    int mul = 1;
                    if ((SCAN_RECEIVED == 1))
                    {   
                        float straight = gl_scan.ranges[24];
                        if(straight == straight){
                            mul += straight; 
                        }
                        if(mul > 2.5){
                            mul = 2.5;
                        }
                        if(straight != straight){
                            mul = 3;
                        }
                        float r = gl_scan.ranges[0];
                        int i = 0;
                        while(r != r){
                            r = gl_scan.ranges[++i];
                        }
                        int j = gl_scan.ranges.size() - 1;
                        float l = gl_scan.ranges[j];
                        while(l != l){
                            l = gl_scan.ranges[--j];
                        }
                        //ROS_ERROR("LEFT = %f , RIGHT = %f",l,r);
                        if(l < r){
                            //ROS_INFO("TURN RIGHT");
                            int minRIGHT = -100;
                            int minLEFT = 100;
                            device.SetCommand(_GO, 1, minRIGHT*mul/2);
                            device.SetCommand(_GO, 2, (minLEFT+(l*100))*mul);
                            //device.SetCommand(_GO, 1, (minRIGHT+(l*100))*mul);
                            //device.SetCommand(_GO, 2, minLEFT*mul);
                        }
                        else if(l > r){
                            //ROS_WARN("TURN LEFT");
                            int minRIGHT = -100;
                            int minLEFT = 100;
                            device.SetCommand(_GO, 1, (minRIGHT-(r*100))*mul);
                            device.SetCommand(_GO, 2, minLEFT*mul/2);
                            //device.SetCommand(_GO, 1, minRIGHT*mul);
                            //device.SetCommand(_GO, 2, (minLEFT-(r*100))*mul);
                        }
                        SCAN_RECEIVED = 0;
                    }
                } 
                //BULLSHIT ENDS HERE         
                 
                    
                    
                    
                    
                }
            }
            device.Disconnect();
            return 0;
        }
~sek_controller(){}    



//DELETE scanCallback
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& str_scan)
{
    //ROS_INFO("SCAN CALLBACK");
    if (SCAN_RECEIVED == 0)
    {
        
        gl_scan.angle_min = str_scan->angle_min;
        //ROS_INFO("FLOAT : %f",str_scan->angle_min);
        //ROS_INFO("FLOAT_2 :%f",scan.angle_min);
        gl_scan.angle_max = str_scan->angle_max;
        gl_scan.range_max = str_scan->range_max;
        gl_scan.range_min = str_scan->range_min;
        gl_scan.angle_increment = str_scan->angle_increment;
        gl_scan.ranges.clear();
        gl_scan.ranges = str_scan->ranges;
        SCAN_RECEIVED = 1;
        
    }
}
};

int main(int argc, char **argv)
{
   
    ros::init(argc, argv, "sek_drive");
    ros::NodeHandle nh;
    sek_controller* sk = 0;
    sk = new sek_controller(nh);
    
    sk->spin();
    return 0;
}
