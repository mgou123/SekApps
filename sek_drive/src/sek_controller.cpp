//#include <stdio.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cmath> 
#include <iostream>
#include <sstream>
#include <fstream>


#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <sek_drive/encoders.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"

#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <chrono>

#include "robodev.cpp"
#include <sdc2130_skel/RoboteqDevice.h>
#include <sdc2130_skel/ErrorCodes.h>
#include <sdc2130_skel/Constants.h>

#include "ros/ros.h" //DELETE

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
        int lenc, renc, lenc_prev, renc_prev, lenc_init, renc_init, enc_errors, lenc_rpm, renc_rpm, lenc_prev_rpm, renc_prev_rpm;
        int MAXRPM;
        double xx, yy, tt, xx_rpm, yy_rpm, tt_rpm, z_vel_imu, zvp;
        
        bool firstOdom;
        bool rp_from_imu;
        bool y_from_imu;
        RoboteqDevice device;
        
        ros::Time prev_time, current_time, enc_loop_time;
        ros::Publisher odom_pub, encoder_pub, encoder_pub_ticks, pose_pub, pose_pub2;
        ros::Subscriber encoder_sub;
        
        ros::Subscriber hok_in;//DELETE
        tf::TransformBroadcaster *odom_broadcaster;
        ros::Subscriber imu_sub;// = n.subscribe("/imu", 10, imu_Callback);

        geometry_msgs::Quaternion q_imu;
        
        std::chrono::time_point<std::chrono::system_clock> p1;
        ofstream file;
        std::stringstream sstream;
        std::string timestamp;
    public:
        sek_controller (ros::NodeHandle& n): n_(n), ODOMETRY_MODE(1), PUB_TF(1), PUB_ODOM(1), CAMERA_ON(0), max_vel_x(0.6), min_vel_x(0.04), xx_rpm(0.0), yy_rpm(0.0), tt_rpm(0.0)
            max_rotational_vel(2.3771), acc_lim_th(0), acc_lim_x(0), acc_lim_y(0), MAX_SPEED_LIMIT(1), MAXRPM(78), RC_MAX_SPEED_LIMIT(0), encoder_ppr(450), encoder_cpr(0), LM(0), RM(0),
            lenc(0), renc(0), lenc_prev(0), renc_prev(0), lenc_init(0), renc_init(0), enc_errors(0), lenc2(0), renc2(0), bullshit(false), rp_from_imu(false),y_from_imu(false), z_vel_imu(0)/*DELETE*/
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
        
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
        {
            int status;
            
            double Left_Motor_v = msg->linear.x - msg->angular.z * (WHEEL_BASE_WIDTH / 2.0);
            double Right_Motor_v = msg->linear.x + msg->angular.z * (WHEEL_BASE_WIDTH / 2.0);
            
            cout<<"Right_Motor_v : "<<Right_Motor_v<<endl;
            cout<<"Left_Motor_v : "<<Left_Motor_v<<endl;
            
            double Left_Motor_rpm = Left_Motor_v *(60.0/(M_PI * 0.15));
            double Right_Motor_rpm = Right_Motor_v *(60.0/(M_PI * 0.15));
            
            cout<<"Right_Motor_rpm : "<<Right_Motor_rpm<<endl;
            cout<<"Left_Motor_rpm : "<<Left_Motor_rpm<<endl;
            

            if (Left_Motor_rpm > MAXRPM)
                Left_Motor_rpm = MAXRPM;
            if (Left_Motor_rpm < - MAXRPM)
                Left_Motor_rpm = -MAXRPM;
            if (Right_Motor_rpm > MAXRPM)
                Right_Motor_rpm = MAXRPM;
            if (Right_Motor_rpm < -MAXRPM)
                Right_Motor_rpm = - MAXRPM;
            
            RM = -Right_Motor_rpm*1000.0/(double)MAXRPM;
            LM = Left_Motor_rpm*1000.0/(double)MAXRPM;
            
            ROS_INFO("MAXRPM : %d", MAXRPM);
            cout<<"RM : "<<RM << endl;
            cout<<"LM : "<<LM << endl;
            
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
            
            //READ ENCODER COUNTER, INCREASES OVER TIME, MORE PRECISE
            if((status = device.GetValue(_C, 1, renc)) !=RQ_SUCCESS){
                cout<<"failed -->"<<status<<endl;
                ros::Duration(5.0).sleep();
                enc_errors++;
            }
            else{
                cout<<"Right Motor Encoder : "<<renc<<endl;
                //ros::Duration(0.1).sleep();
            }
            if((status = device.GetValue(_C, 2, lenc)) !=RQ_SUCCESS){
                cout<<"failed -->"<<status<<endl;
                ros::Duration(5.0).sleep();
                enc_errors++;
            }
            else{
                cout<<"Left Motor Encoder : "<<lenc<<endl;
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
            
            //READ RPM FROM WHEELS    
                
                if((status = device.GetValue(_ABSPEED, 1, renc_rpm)) !=RQ_SUCCESS)
                {
                    cout<<"failed -->"<<status<<endl;
                    ros::Duration(5.0).sleep();
                    enc_errors++;
                }
                else
                {
                    cout<<"Right Motor RPM : "<<renc_rpm<<endl;
                    ros::Duration(0.01).sleep();
                }
                if((status = device.GetValue(_ABSPEED, 2, lenc_rpm)) !=RQ_SUCCESS)
                {
                    cout<<"failed -->"<<status<<endl;
                    ros::Duration(5.0).sleep();
                    enc_errors++;
                }
                else
                {
                    cout<<"Left Motor RPM : "<<lenc_rpm<<endl;
                    ros::Duration(0.01).sleep();
                }
                renc_rpm = -renc_rpm;
                /*
                sek_drive::encoders encoder_msg;
                encoder_msg.header.stamp = now;
                encoder_msg.header.frame_id = "base_link";
                encoder_msg.time_delta = delta;
                encoder_msg.left_wheel = lenc2;
                encoder_msg.right_wheel = renc2;
                encoder_pub.publish(encoder_msg);
                */
        }
        
        void calcVoltage(){
            int internalVoltage[3] ;
            int batteryAmps[3];
            int status = -1;
            int bat1;
            int bat2;
            int bat3;
            int amps1;
            int amps2;
            
            sstream<<std::chrono::duration_cast<std::chrono::milliseconds>(
            p1.time_since_epoch()).count();
            timestamp = sstream.str();
            file<<timestamp<<" ";
            timestamp.clear();
            sstream.str(std::string());
            sstream.clear();
            status = device.GetValue(_VOLTS, 1, bat1);
            if(status == RQ_SUCCESS){
                //cout<<"Status : "<<status<<endl;
                //cout<<"Internal Voltage : "<<(float)bat1<<endl;
                file<<(float)bat1<<" ";
            }
            else{
                //ROS_ERROR("Error reading battery voltage");
            }
            
            status = -1;
            status = device.GetValue(_VOLTS, 2, bat2);
            if(status == RQ_SUCCESS){
                //cout<<"Status : "<<status<<endl;
                //cout<<"Main Battery Voltage : "<<(float)bat2/10<<endl;
                file<<(float)bat2<<" ";
            }
            else{
                //ROS_ERROR("Error reading battery voltage");
            }
            
            status = -1;
            status = device.GetValue(_VOLTS, 3, bat3);
            if(status == RQ_SUCCESS){
                //cout<<"Status : "<<status<<endl;
                //cout<<"5V output on DSub connector in millivolts : "<<(float)bat3<<endl;
                file<<(float)bat3<<" ";
            }
            else{
               // ROS_ERROR("Error reading battery voltage");
            }
            
            status = -1;
            status = device.GetValue(_MOTAMPS, 1, amps1);
            if(status == RQ_SUCCESS){
                //cout<<"Status : "<<status<<endl;
                //cout<<"Motor 1 Amps : "<<(float)amps1/10<<endl;
                file<<(float)amps1<<" ";
            }
            else{
                //ROS_ERROR("Error reading amps");
            }
            
            status = -1;
            status = device.GetValue(_MOTAMPS, amps2);
            if(status == RQ_SUCCESS){
                //cout<<"Status : "<<status<<endl;
                //cout<<"Motor 2 Amps : "<<(float)amps2/10<<endl;
                file<<(float)amps2<<" ";
            }
            else{
                //ROS_ERROR("Error reading amps");
            }
            
            status = -1;
            int renc = 0;
            if((status = device.GetValue(_C, 1, renc)) !=RQ_SUCCESS){
                cout<<"failed -->"<<status<<endl;
                ros::Duration(5.0).sleep();
                //enc_errors++;
            }
            else{
                //cout<<"Right Motor Encoder Ticks : "<<renc<<endl;
                //ros::Duration(0.1).sleep();
                file<<renc<<" ";
            }
            
            status = -1;
            lenc = 0;
            if((status = device.GetValue(_C, 2, lenc)) !=RQ_SUCCESS){
                cout<<"failed -->"<<status<<endl;
                ros::Duration(5.0).sleep();
                //enc_errors++;
            }
            else{
                //cout<<"Left Motor Encoder Ticks : "<<lenc<<endl;
                //ros::Duration(0.1).sleep();
                file<<lenc<<" ";
            }
            
            status = -1;
            int r_rpm = 0;
            if((status = device.GetValue(_S, 1, r_rpm)) !=RQ_SUCCESS){
                cout<<"failed -->"<<status<<endl;
                ros::Duration(5.0).sleep();
                //enc_errors++;
            }
            else{
                //cout<<"Right Motor Encoder RPM : "<<r_rpm<<endl;
                //ros::Duration(0.1).sleep();
                file<<r_rpm<<" ";
            }
            
            status = -1;
            int l_rpm = 0;
            if((status = device.GetValue(_S, 2, l_rpm)) !=RQ_SUCCESS){
                cout<<"failed -->"<<status<<endl;
                ros::Duration(5.0).sleep();
                //enc_errors++;
            }
            else{
                //cout<<"Left Motor Encoder RPM : "<<l_rpm<<endl;
                //ros::Duration(0.1).sleep();
                file<<l_rpm<<" ";
            }
            file<<endl;
            //cout<<"******************************"<<endl;
            //ros::Duration(1.0).sleep();
            
            /*
            status = 0;
            if ((status = device.GetValue(_VOLTS, 0, batteryAmps)) !=RQ_SUCCESS){
                ROS_INFO("Bat Amps : %d", internalVoltage);
            }                               
            else{
                ROS_INFO("Status : %d", status);
                ROS_INFO("Error Reading Battery Amps");
                ROS_INFO("Battery Amps : %d", batteryAmps);
            }
            * */
        }
        
        void calcOdom()
        {
            //cout<<"Odometry Callback"<<endl;
            ros::Time now = ros::Time::now();
            double delta_time = (now - prev_time).toSec();
            prev_time = now;
            //ROS_INFO("Delta Time : 
            if (firstOdom)
            {
                renc_prev = renc;
                lenc_prev = renc_prev;
                zvp = z_vel_imu;
            }
            
            int diff_lenc = lenc - lenc_prev;
            int diff_renc = renc - renc_prev;
            //ROS_INFO("Diff renc : %d",diff_renc);
            //ROS_INFO("Diff lenc : %d",diff_lenc);
            //ros::Duration(0.5).sleep();
            renc_prev = renc;
            lenc_prev = lenc;
            
            
            //ODOM FROM ENCODER TICKS
            double l_w = diff_lenc * 2.0 * PI / (double)encoder_cpr / delta_time;
            double r_w = diff_renc * 2.0 * PI / (double)encoder_cpr / delta_time;
            //ROS_INFO("L_W : %f     R_W : %f",l_w ,r_w);
            
            double l_v = l_w * DIAMETER / 2.0;
            double r_v = r_w * DIAMETER / 2.0;
            //ROS_INFO("L_V : %f     R_V : %f",l_v ,r_v);
            double v = double((l_v + r_v) / 2.0);
            //ROS_INFO("V AFTER : %f",v);
            double w = double((r_v - l_v) / WHEEL_BASE_WIDTH);
            tt = tt + w*delta_time;
            tt = wrapToPi(tt);
            //ODOM FORM ENCODER TICKS
            
            
            //ODOM FROM ENCODER RPM
            
            //ODOM FROM ENCODER RPM
            //
            /*
            if (!rp_from_imu) {
                    roll = 0;
                    pitch = 0;
            }
            //	tt=wrapToPi(yaw);
            //}
           if (y_from_imu) {
                tt = wrapToPi(yaw);
            }
            else {
                    tt = tt + w * delta_time;
                    tt = wrapToPi(tt);
            }
            */
            geometry_msgs::Quaternion quat;
            double roll = 0, pitch = 0, yaw = 0;
            tf::Quaternion q;
            tf::quaternionMsgToTF(q_imu, q);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            //ROS_INFO("%f", yaw);
            
            /*ODOM FRON IMU
            if (firstOdom){
                xx = 0;
                yy = 0;
                tt = 0;
                firstOdom = false;
            }else{
                roll = 0;
                pitch = 0;
                tt = wrapToPi(yaw);
            }
            */
            
            quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, tt);
            //quat = tf::createQuaternionMsgFromRollPitchYaw(wrapToPi(roll), wrapToPi(pitch), tt);
            
            //ROS_INFO(" sdfgasdgasdga %f",v*cos(tt));
            xx = xx + (v * cos(tt)) * delta_time ;
            yy = yy + (v * sin(tt)) * delta_time;
            //ROS_INFO("%f   %f",xx ,yy);
            //ROS_INFO("%f    %f",v ,w);
            //ROS_INFO("---------");
            nav_msgs::Odometry odom_msg;
            //odom_msg.header.stamp = now;
            odom_msg.header.frame_id = "odom";
            odom_msg.header.stamp = now;
            odom_msg.pose.pose.position.x = xx;
            //ROS_INFO("%f",xx);
            odom_msg.pose.pose.position.y = yy;
            //ROS_INFO("%f",yy);
            //ROS_INFO("%f", w);
            odom_msg.pose.pose.position.z = 0;
            odom_msg.pose.pose.orientation = quat;
            odom_msg.child_frame_id = "base_link";
            odom_msg.twist.twist.linear.x = v;
            odom_msg.twist.twist.linear.y = 0;
            //odom_msg.twist.twist.angular.z = z_vel_imu - zvp;
            //zvp = z_vel_imu;
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
            
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = now;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";
            odom_trans.transform.translation.x = xx;
            odom_trans.transform.translation.y = yy;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = quat;
            odom_broadcaster->sendTransform(odom_trans);
            /*
            else
            {
                double l_v = (2*M_PI*(DIAMETER/2.0)*lenc2)/60;
                double r_v = (2*M_PI*(DIAMETER/2.0)*renc2)/60;
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
                else {
                    if (!rp_from_imu) {
                        roll = 0;
                        pitch = 0;
                }
                //	tt=wrapToPi(yaw);
                //}
                    if (y_from_imu) {
                        tt = wrapToPi(yaw);
                    }
                    else {
                        tt = tt + w * delta_time;
                        tt = wrapToPi(tt);
                    }
                    xx = xx + (v * cos(tt)) * delta_time ;
                    yy = yy + (v * sin(tt)) * delta_time;
                
                    //ROS_INFO("TT BEFORE WRAP : %f",tt);
                
                    quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, tt);
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
                */
                
            
                
            }
        
        //void imu_Callback(const sensor_msgs::Imu::ConstPtr& msg) {
        void recordCallback(const sensor_msgs::Imu::ConstPtr& msg){
            int internalVoltage[3] ;
            int batteryAmps[3];
            int status = -1;
            int bat1;
            int bat2;
            int bat3;
            int amps1;
            int amps2;
            
            sstream<<std::chrono::duration_cast<std::chrono::milliseconds>(
            p1.time_since_epoch()).count();
            timestamp = sstream.str();
            file<<timestamp<<" ";
            timestamp.clear();
            sstream.str(std::string());
            sstream.clear();
            status = device.GetValue(_VOLTS, 1, bat1);
            if(status == RQ_SUCCESS){
                //cout<<"Status : "<<status<<endl;
                //cout<<"Internal Voltage : "<<(float)bat1<<endl;
                file<<(float)bat1<<" ";
            }
            else{
                //ROS_ERROR("Error reading battery voltage");
            }
            
            status = -1;
            status = device.GetValue(_VOLTS, 2, bat2);
            if(status == RQ_SUCCESS){
                //cout<<"Status : "<<status<<endl;
                //cout<<"Main Battery Voltage : "<<(float)bat2/10<<endl;
                file<<(float)bat2<<" ";
            }
            else{
                //ROS_ERROR("Error reading battery voltage");
            }
            
            status = -1;
            status = device.GetValue(_VOLTS, 3, bat3);
            if(status == RQ_SUCCESS){
                //cout<<"Status : "<<status<<endl;
                //cout<<"5V output on DSub connector in millivolts : "<<(float)bat3<<endl;
                file<<(float)bat3<<" ";
            }
            else{
               // ROS_ERROR("Error reading battery voltage");
            }
            
            status = -1;
            status = device.GetValue(_MOTAMPS, 1, amps1);
            if(status == RQ_SUCCESS){
                //cout<<"Status : "<<status<<endl;
                //cout<<"Motor 1 Amps : "<<(float)amps1/10<<endl;
                file<<(float)amps1<<" ";
            }
            else{
                //ROS_ERROR("Error reading amps");
            }
            
            status = -1;
            status = device.GetValue(_MOTAMPS, amps2);
            if(status == RQ_SUCCESS){
                //cout<<"Status : "<<status<<endl;
                //cout<<"Motor 2 Amps : "<<(float)amps2/10<<endl;
                file<<(float)amps2<<" ";
            }
            else{
                //ROS_ERROR("Error reading amps");
            }
            
            status = -1;
            int renc = 0;
            if((status = device.GetValue(_C, 1, renc)) !=RQ_SUCCESS){
                file<<"failed -->"<<status<<endl;
                ros::Duration(5.0).sleep();
                //enc_errors++;
            }
            else{
                //cout<<"Right Motor Encoder Ticks : "<<renc<<endl;
                //ros::Duration(0.1).sleep();
                file<<renc<<" ";
            }
            
            status = -1;
            lenc = 0;
            if((status = device.GetValue(_C, 2, lenc)) !=RQ_SUCCESS){
                cout<<"failed -->"<<status<<endl;
                ros::Duration(5.0).sleep();
                //enc_errors++;
            }
            else{
                //cout<<"Left Motor Encoder Ticks : "<<lenc<<endl;
                //ros::Duration(0.1).sleep();
                file<<lenc<<" ";
            }
            
            status = -1;
            int r_rpm = 0;
            if((status = device.GetValue(_S, 1, r_rpm)) !=RQ_SUCCESS){
                file<<"failed -->"<<status<<endl;
                ros::Duration(5.0).sleep();
                //enc_errors++;
            }
            else{
                //cout<<"Right Motor Encoder RPM : "<<r_rpm<<endl;
                //ros::Duration(0.1).sleep();
                file<<r_rpm<<" ";
            }
            
            status = -1;
            int l_rpm = 0;
            if((status = device.GetValue(_S, 2, l_rpm)) !=RQ_SUCCESS){
                file<<"failed -->"<<status<<endl;
                ros::Duration(5.0).sleep();
                //enc_errors++;
            }
            else{
                //cout<<"Left Motor Encoder RPM : "<<l_rpm<<endl;
                //ros::Duration(0.1).sleep();
                file<<l_rpm<<" ";
            }
            
            //cout<<"******************************"<<endl;
            //ros::Duration(1.0).sleep();
            
            //IMU LOGGING
            file<<msg->linear_acceleration.x<<" ";
            file<<msg->linear_acceleration.y<<" ";
            file<<msg->linear_acceleration.z<<" ";
            file<<msg->angular_velocity.x<<" ";
            file<<msg->angular_velocity.y<<" ";
            file<<msg->angular_velocity.z<<" ";
            file<<msg->orientation.x<<" ";
            file<<msg->orientation.y<<" ";
            file<<msg->orientation.z<<" ";
            file<<msg->orientation.w<<" ";
            file<<endl;
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
            q_imu = tf::createQuaternionMsgFromYaw(0);
            
            n_.param("roll_pitch_from_imu", rp_from_imu, true);
            n_.param("yaw_from_imu", y_from_imu, true);
            //imu_sub = n_.subscribe("/imu", 10, &sek_controller::recordCallback, this);
    
            ros::Subscriber sub = n_.subscribe("/joy", 1, &sek_controller::teleopCallback, this);
            
            
            hok_in = n_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &sek_controller::scanCallback, this);//DELETE
            
            //ros::Subscriber align_sub = n_.subscribe("/motor_commands", 1, &sek_controller::alignCallback, this);
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
            
            //ROS_INFO("Reading RPM");
            if((status = device.GetConfig(_MXRPM, 1, MAXRPM)) !=RQ_SUCCESS)
                cout<<"reading rpm M1 failed -->"<<status<<endl;
            else
                cout<<"MOTOR 1 MAXRPM : "<<MAXRPM<<endl;
                
            if((status = device.GetConfig(_MXRPM, 2, MAXRPM)) !=RQ_SUCCESS)
                cout<<"reading rpm M2 failed -->"<<status<<endl;
            else
                cout<<"MOTOR 2 MAXRPM : "<<MAXRPM<<endl;
            
            encoder_cpr = encoder_ppr*4;
            
            printf ("Sek Operational\n\n");
            ros::Duration(0.01).sleep(); //sleep for 10 ms
            
            
            //SKATA    
            SCAN_RECEIVED = 0;
            float min_angle = gl_scan.angle_min;//save the minimum angle in degrees
            float max_angle = gl_scan.angle_max;//save the maximum angle in degrees
            //SKATA
            
            //p1 = std::chrono::system_clock::now();
            //cout<<"gfdsgsdfhsfdhsdfhsfdhsdf"<<endl;
            //ROS_INFO("ASDKJHGADSLJHGLAKSDJH");
            file.open("/home/skel/sek_logger.txt", ios::out | ios::binary);
            file<<"Time   Volts1 Volts2 Volts3 Amps1 Amps2 R_Ticks L_Ticks R_rpm L_rpm"<<endl;
            
            bool first = true;
            while (ros::ok())
            {
                ros::spinOnce();
                current_time = ros::Time::now();
                if ((current_time.toSec() - enc_loop_time.toSec())>=0.2)
                {
                    
                    readEnc();
                    calcOdom();
                    //calcVoltage();
                    //pub2.publish(motor_commands);
                    enc_loop_time = current_time;
                    
                    
                    
                //BULLSHIT STARTS HERE  
                //ROS_INFO("Looping");
                if(bullshit){
                    int mul = 1;
                    if ((SCAN_RECEIVED == 1))
                    {   
                        float straight = gl_scan.ranges[floor(gl_scan.ranges.size()/2)];
                        if(straight == straight){
                            mul += straight/2; 
                        }
                        if(mul > 2){
                            mul = 2;
                        }
                        float r = gl_scan.ranges[120];
                        int i = 0;
                        while((r != r) || (r>gl_scan.range_max) || (r<gl_scan.range_min)){
                            
                            r = gl_scan.ranges[++i];
                        }
                        int j = gl_scan.ranges.size() - 121;
                        float l = gl_scan.ranges[j];
                        while ((l != l) || (l>gl_scan.range_max) || (l<gl_scan.range_min)){
                            l = gl_scan.ranges[--j];
                        }
                        
                    if(abs(l-r) > 0.15 || (l < 0.45 && l > 0.25 && r > 0.25 && r < 0.45)){
                        if(l < r){
                            //ROS_INFO("TURN RIGHT");
                            if(l < 0.35){
                                device.SetCommand(_GO, 1, 300*mul);
                                device.SetCommand(_GO, 2, 400*mul);
                            }
                            else if(l < 0.45){
                                device.SetCommand(_GO, 1, -200*mul);
                                device.SetCommand(_GO, 2, 500*mul);
                            }
                            else if(l < 0.7){
                                device.SetCommand(_GO, 1, -300*mul);
                                device.SetCommand(_GO, 2, 600*mul);
                            }
                            else if(l < 1.5){
                                device.SetCommand(_GO, 1, -450*mul);
                                device.SetCommand(_GO, 2, 900*mul);
                            }
                            else{
                                device.SetCommand(_GO, 1, -600*mul);
                                device.SetCommand(_GO, 2, 1000*mul);
                            }
                        }
                        else if(l > r){
                            //ROS_WARN("TURN LEFT");
                            if(r< 0.35){
                                device.SetCommand(_GO, 1, -400*mul);
                                device.SetCommand(_GO, 2, -300*mul);
                            }
                            else if(r < 0.45){
                                device.SetCommand(_GO, 1, -500*mul);
                                device.SetCommand(_GO, 2, 200*mul);
                            }
                            else if(r < 0.7){
                                device.SetCommand(_GO, 1, -600*mul);
                                device.SetCommand(_GO, 2, 300*mul);
                            }
                            else if(r < 1.5){
                                device.SetCommand(_GO, 1, -900*mul);
                                device.SetCommand(_GO, 2, 450*mul);
                            }
                            else{
                                device.SetCommand(_GO, 1, -1000*mul);
                                device.SetCommand(_GO, 2, 600*mul);
                            }
                        }
                    }
                    else{
                        if(straight >= 1.5){
                            device.SetCommand(_GO, 1, -700*mul);
                            device.SetCommand(_GO, 2, 700*mul);
                        }
                        else if(straight >= 1){
                            device.SetCommand(_GO, 1, -600*mul);
                            device.SetCommand(_GO, 2, 600*mul);
                        }
                        else{
                            device.SetCommand(_GO, 1, -500*mul);
                            device.SetCommand(_GO, 2, 500*mul);
                        }
                    }
                        
                       /* float r = gl_scan.ranges[0];
                        int i = 0;
                        while(r != r){
                            r = gl_scan.ranges[++i];
                        }
                        int j = gl_scan.ranges.size() - 1;
                        float l = gl_scan.ranges[j];
                        while(l != l){
                            l = gl_scan.ranges[--j];
                        }
                        //ROS_INFO("L = %f",l);
                        //ROS_INFO("R = %f",r);
                        if(l > r){//turn left
                                if(l < 0.4){
                                    device.SetCommand(_GO, 1, -400);
                                    device.SetCommand(_GO, 2, 100);
                                }
                                else if(l < 0.6){
                                    device.SetCommand(_GO, 1, -600);
                                    device.SetCommand(_GO, 2, 100);
                                }
                                else if(l < 0.8){
                                    device.SetCommand(_GO, 1, -700);
                                    device.SetCommand(_GO, 2, 200);
                                }
                                else if(l < 1){
                                    device.SetCommand(_GO, 1, -800);
                                    device.SetCommand(_GO, 2, 300);
                                }
                                else{
                                    device.SetCommand(_GO, 1, -800);
                                    device.SetCommand(_GO, 2, 400);
                                }
                        }
                        else{//turn right
                            if(r < 0.4){
                                device.SetCommand(_GO, 1, -100);
                                device.SetCommand(_GO, 2, 400);
                            }
                            else if(r < 0.6){
                                device.SetCommand(_GO, 1, -100);
                                device.SetCommand(_GO, 2, 600);
                            }
                            else if(r < 0.8){
                                device.SetCommand(_GO, 1, -200);
                                device.SetCommand(_GO, 2, 700);
                            }
                            else if(r < 1){
                                device.SetCommand(_GO, 1, -300);
                                device.SetCommand(_GO, 2, 800);
                            }
                            else{
                                device.SetCommand(_GO, 1, -400);
                                device.SetCommand(_GO, 2, 800);
                            }
                        }*/
                    SCAN_RECEIVED = 0;
                    }
                } 
                //BULLSHIT ENDS HERE         
                }
            }
            file<<endl<<"END"<<endl;
            file.close();
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
