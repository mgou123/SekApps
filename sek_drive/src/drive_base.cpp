#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//XARALAMPOS
#include <std_msgs/Int32MultiArray.h>
std_msgs::Int32MultiArray motor_commands;
//

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

#define PI 3.14159265358979323846264338
#define DIAMETER 0.1524 //m,6 INCHES
#define WHEEL_BASE_WIDTH 0.40 //m,  
#define TWOPI /*360*/6.2831853070
#define RADS 57.2958
#define MAX_SPEED 800 //command
#define MAX_RPM 360	//RPM
#define MAX_LIN_VEL  2
#define MIN_LIN_VEL  -2
#define MAX_ROT_VEL  4
#define MIN_ROT_VEL  -4

/*                                            
double circumference = PI* DIAMETER; //0.47877872040708448954
double max_lin_vel = circumference*(MAX_SPEED/60); //2.87267232244250693724
double max_ang_vle = max_lin_vel/(WHEEL_BASE_WIDTH/2);//0.14363361612212534686
int six = 0;
int gyro_speed = 0;
*/

double max_lin = 2;
double min_lin = -2;
double max_rot = 4;
double min_rot = - 4;
double c__1=2/(max_lin - min_lin);
double c__2=2/(max_rot - min_rot);
int LM = 0;
int RM = 0;
int HB = 0; //handbrake signal
int ESD = 0; //emergency shutdown signal
int MODE = 0; //Assistive or nope
int REC = 0;
int FIRST_POSE_SET = 0;
int PLAYING = 0;
int CAMERA_ON = 0;
int SCAN_RECORD = 0;
int ALLIGNING = 0;
int status;
string response = "";
RoboteqDevice device;
int i ;	
int ping_ret,p_status;
int sign = 0;
int speed;	
int count_ = 0;
int lenc = 0;
int renc = 0;

double posx = 0.0;
double posx_prev = 0.0;
double posy = 0.0;
double posy_prev = 0.0;
double posth = 0.0;
double linx = 0.0;
double angz = 0.0;
double linx_prev = 0.0;
double angz_prev = 0.0;
int battery = 0;

ros::Time current_time, last_time, total_time;

ofstream file;

unsigned int u;
FILE *output;
int result_2;

struct pose_coords
{
    float x;
    float y;
    float w;
};

struct pose_coords first_pose;
std::vector<pose_coords> pose_vector;
std::vector<pose_coords> pose_vector_to_use;
nav_msgs::Path path_to_send;


float scaleRange(double in, double oldMin, double oldMax, double newMin, double newMax)
{
    return (in / ((oldMax - oldMin) / (newMax - newMin))) + newMin;
}

void recManCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    
    if (REC == 1)
    {   
        if (FIRST_POSE_SET == 0)
        {
            first_pose.x = msg->pose.pose.position.x;
            first_pose.y = msg->pose.pose.position.y;
            //pose.z = msg->pose.orientation.z;
            first_pose.w = msg->pose.pose.orientation.w;
            FIRST_POSE_SET = 1;
        }
        struct pose_coords pose ;
        pose.x = msg->pose.pose.position.x - first_pose.x;
        pose.y = msg->pose.pose.position.y - first_pose.y;
        pose.w = msg->pose.pose.orientation.w - first_pose.w;
        pose_vector.push_back(pose);
    }
}

void teleopCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if(MODE == 1)
    {
        //TODO http://www.dandwiki.com/wiki/SRD:Find_the_Path
        return ;
    } 
    else
    {
    if((msg->axes[5]!=0)||(msg->axes[6]!=0)||(msg->axes[0]!=0)||(msg->axes[1]!=0)||(msg->axes[4]!=0)||(msg->axes[3]!=0))
    {   
        cout<<"if"<<endl;
        //ROS_INFO("HEAR HEAR");
        if(msg->axes[6]==1)//MPROS-PISW
        {
            RM = -MAX_SPEED;
            LM = MAX_SPEED;
            //gyro_speed = LM ;
        }
        else if(msg->axes[6]==-1)
        {
            RM = MAX_SPEED;
            LM = -MAX_SPEED;
            //gyro_speed = LM ;
            //system("mplayer -really-quiet /home/skel/beep2.mp3&");
            
        }
        else if(msg->axes[5]!=0)//DEKSIA-ARISTERA
        {
            RM = -MAX_SPEED*msg->axes[5];
            LM = -MAX_SPEED*msg->axes[5];
        }
        else if((msg->axes[0]!=0)||(msg->axes[1]!=0))
        {
            if(msg->axes[1]==0)//akinito oxhma
            {
                if(msg->axes[0]!=0)//epitopia peristrofi
                {
                    RM = -MAX_SPEED*msg->axes[0];
                    LM = -MAX_SPEED*msg->axes[0];
                }
            }
            else
            {
                RM = -MAX_SPEED*msg->axes[1];
                LM = MAX_SPEED*msg->axes[1];
                if(msg->axes[1]>0)//kinisi mprosta
                {   
                    if(msg->axes[0]>0)//strofi aristera
                    {
                        RM = RM + RM*msg->axes[0];
                        if (RM<(-MAX_SPEED))
                        {
                            RM=-MAX_SPEED;
                        }
                        LM = LM - LM*msg->axes[0];
                        if (LM<(0.2*(MAX_SPEED)))
                        {
                            LM=0.2*(MAX_SPEED);
                        }
                        
                    }
                    else if (msg->axes[0]<0)//strofi de3ia
                    {
                        LM = LM - LM*msg->axes[0];
                        if (LM>(MAX_SPEED))
                        {
                            LM=MAX_SPEED;
                        }
                        RM = RM + RM*msg->axes[0];
                        if (RM>(-0.2*(MAX_SPEED)))
                        {
                            RM=-0.2*(MAX_SPEED);
                        }
                        
                    }
                }
                else if (msg->axes[1]<0)//kinisi pisw
                {   
                    RM = -MAX_SPEED*msg->axes[1]; //>0
                    LM = MAX_SPEED*msg->axes[1]; //<0
                    if(msg->axes[0]>0)//strofi aristera
                    {
                        RM = RM + RM*msg->axes[0];
                        if (RM>(MAX_SPEED))
                        {
                            RM=MAX_SPEED;
                        }
                        LM = LM - LM*msg->axes[0];
                        if (LM>(-0.2*(MAX_SPEED)))
                        {
                            LM=-0.2*(MAX_SPEED);
                        }
                        
                    }
                    else if (msg->axes[0]<0)//strofi de3ia
                    {
                        LM = LM - LM*msg->axes[0];
                        if (LM<(-MAX_SPEED))
                        {
                            LM = -MAX_SPEED;
                        }
                        RM = RM + RM*msg->axes[0];
                        if (RM<(0.2*(MAX_SPEED)))
                        {
                            RM=0.2*(MAX_SPEED);
                        }
                        
                    }
                }
            }
        }
        device.SetCommand(_GO,1, RM);// RIGHT
        device.SetCommand(_GO,2, LM);//LEFT
        cout<<"LEFT MOTOR :"<< LM<<endl;
        cout<<"RIGHT MOTOR :"<<RM<<endl;
        motor_commands.data.clear();
        motor_commands.data.push_back(RM);
        motor_commands.data.push_back(LM);
        //calcOdom();
        if(msg->buttons[0]==1)
        {
            //system("mplayer -really-quiet /home/skel/horn.mp3 &");
            //ros::Duration(2).sleep();
            //system("killall -9 mplayer");
        }
        if(msg->buttons[3]==1)
        {
            ROS_INFO("SAVING MAP as \"mymap\"");
            system("rsrun map_server map_saver -f mymap &");
        }
    }
    else 
    {
        cout<<"else"<<endl;
        //system("killall -9 mplayer");
        device.SetCommand(_GO,1, 0);// RIGHT
        device.SetCommand(_GO,2, 0);//LEFT
        motor_commands.data.clear();
        motor_commands.data.push_back(0);
        motor_commands.data.push_back(0);
        /*
        if((msg->buttons[11])==1)//&&(msg->buttons[9]!=1))
        {
            if (SCAN_RECORD==0)
            {
                system("roslaunch sek_drive record.launch &");
                ros::Duration(3).sleep();
                system("/home/skel/start");;
                SCAN_RECORD = 1;
            }
            else
            {
                system("rosnode kill /laser_scan/record_1  &");
                system("rosnode kill /laser_scan/record_2  &");
                system("rosnode kill /laser_scan/record_3  &");	
                system("rosnode kill /laser_scan/record_4  &");
                system("/home/skel/stop");;
                ros::Duration(3).sleep();
                system("rosnode kill /laser_scan/img_record  &");
                SCAN_RECORD = 0;
            }
            ros::Duration(2).sleep();
            //system("killall -9 mplayer");
        }
        */
        if(msg->buttons[1]==1)
        {
            system("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID '{}' &");
        }
        if(msg->buttons[3]==1)
        {
            ROS_INFO("SAVING MAP as \"mymap\"");
            system("rosrun map_server map_saver -f mymap &");
        }
        if(msg->buttons[2]==1)
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
                pose_vector_to_use =  pose_vector;
                pose_vector.clear();
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
                for(unsigned i = 0; i < pose_vector.size(); i++)
                {
                    pose_to_add.pose.position.x = pose_vector[i].x;
                    pose_to_add.pose.position.y = pose_vector[i].y;
                    pose_to_add.pose.orientation.w = pose_vector[i].w;
                    path_to_send.poses.push_back(pose_to_add);
                }
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
    //cout<<"RENC : "<<renc<<endl;
    //cout<<"LENC : "<<lenc<<endl;
}
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg2)
{
    linx = msg2->linear.x*0.250000;
    angz = msg2->angular.z*0.250000;
    
    if(linx==0 && angz ==0)
    {
        device.SetCommand(_GO,1, 0);// RIGHT
        device.SetCommand(_GO,2, 0);//LEFT
        return;
    }
    else
    {
        if ((abs(linx)<0.2)&&(abs(linx)>0))
        {
            linx = (linx/(abs(linx)))*0.2;
        }
        if ((abs(angz) <0.2)&&(abs(angz)>0))
        {
            angz = (angz/(abs(angz)))*0.2;
        }
    }
    //ROS_INFO("LINX #2 : %f",linx);
    //ROS_INFO("ANGZ #2 : %f",angz);
    if(linx==0)//akinito oxhma
    {
        if(angz!=0)//epitopia peristrofi
        {
            RM = -MAX_SPEED*angz;
            LM = -MAX_SPEED*angz;
            if(abs(RM)<(0.2*MAX_SPEED))
            {
                RM = (RM/(abs(RM)))*(0.2*MAX_SPEED);
                LM = (LM/(abs(LM)))*(0.2*MAX_SPEED);
            }
        }
    }
    else
    {   
        RM = -MAX_SPEED*linx;
        LM = MAX_SPEED*linx;
        if(linx>0)//kinisi mprosta
        {   
            if(angz>0)//strofi aristera
            {
                ROS_INFO("STROFI MPROSTA ARISTERA");
                if(angz>0.8)//apotomi strofi
                {
                    ROS_INFO("APOTOMA");
                    RM = RM*1.8;
                    if (RM<MAX_SPEED)
                    {
                        RM=-MAX_SPEED;
                    }
                    LM = LM*angz;
                    if (LM<(0.2*(MAX_SPEED)))
                    {
                        LM=0.2*(MAX_SPEED);
                    }
                }
                else
                {
                    ROS_INFO("NORMAL");
                    RM = RM + RM*angz;
                    if (RM<(-MAX_SPEED))
                    {
                        RM=-MAX_SPEED;
                    }
                    LM = LM - LM*angz;
                    if (LM<(0.2*(MAX_SPEED)))
                    {
                        LM=0.2*(MAX_SPEED);
                    }
                }
        }
        else if (angz<0)//strofi de3ia
        {
            ROS_INFO("STROFI MPROSTA DE3IA");
            if(angz<(-0.8))//apotomi strofi
            {
                ROS_INFO("APOTOMA");
                LM = LM*1.8;
                if (LM>MAX_SPEED)
                {
                    LM=MAX_SPEED;
                }
                //RM<0
                //msg-axes[0] <0
                RM = RM - RM*angz;
                if (RM<(-0.2*(MAX_SPEED)))
                {
                    RM=-0.2*(MAX_SPEED);
                }
            }
            else
            {   
                ROS_INFO("NORMAL");
                LM = - LM*angz;
                if (LM<(0.2*MAX_SPEED))
                {
                    LM=0.2*MAX_SPEED;
                }
                RM = RM + RM*angz;
                if (RM<(-0.2*(MAX_SPEED)))
                {
                    RM=-0.2*(MAX_SPEED);
                }
            }
        }
        else //kamia strofi
        {
            if(LM<(0.2*MAX_SPEED))
            {
                LM=0.2*MAX_SPEED;
                RM=-0.2*MAX_SPEED;
            } 
        }
    }
    else if (linx<0)//kinisi pisw
    {   
        RM = -MAX_SPEED*linx; //>0
        LM = MAX_SPEED*linx; //<0
        if(angz>0)//strofi aristera
        {
            ROS_INFO("STROFI PISW ARISTERA");
            if(angz>0.8)//apotomi strofi
            {
                //axes[0]>0
                //RM >0
                //theloyme na ay3h8ei to RM
                ROS_INFO("APOTOMA");
                RM = RM*1.8;
                if (RM>MAX_SPEED)
                {
                    RM=MAX_SPEED;
                }
                //LM <0
                //angz > 0
                //theloyme na meiw8ei to LM
                LM = LM + LM*angz;
                if (LM>(-0.2*(MAX_SPEED)))
                {
                    LM=-0.2*(MAX_SPEED);
                }
            }
            else
            {
                //axes[0]>0
                //RM >0
                //theloyme na ay3h8ei to RM
                ROS_INFO("NORMAL");
                RM = RM + RM*angz;
                if (RM>MAX_SPEED)
                {
                    RM=MAX_SPEED;
                }
                //LM <0
                //angz > 0
                //theloyme na meiw8ei to LM
                LM = LM - LM*angz;
                if (LM>(-0.2*(MAX_SPEED)))
                {
                    LM=-0.2*(MAX_SPEED);
                }
            }
        }
        else if (angz<0)//strofi de3ia
        {
            ROS_INFO("STROFI PISW DE3IA");
            //RM > 0 8eloyme na meiw8ei
            //LM < 0 8eloyme na af3i8ei
            //angz < 0
            if(angz<(-0.8))//apotomi strofi
            {
                ROS_INFO("APOTOMA");
                LM = LM*1.8;
                if (LM<-(MAX_SPEED))
                {
                    LM=-MAX_SPEED;
                }
                RM = RM + RM*angz;
                if (RM<(0.2*(MAX_SPEED)))
                {
                    RM=0.2*(MAX_SPEED);
                }
            }
            else
            {   
                ROS_INFO("NORMAL");
                LM = LM - LM*angz;
                if (LM<-(MAX_SPEED))
                {
                    LM=-MAX_SPEED;
                }
                RM = RM + RM*angz;
                if (RM<(0.2*(MAX_SPEED)))
                {
                    RM=0.2*(MAX_SPEED);
                }
            }
        }
        else //kamia strofi
        {
            if(RM<(0.2*MAX_SPEED))
            {
                RM=0.2*MAX_SPEED;
                LM=-0.2*MAX_SPEED;
            } 
        }
    }
}
device.SetCommand(_GO,1, RM);// RIGHT
device.SetCommand(_GO,2, LM);//LEFT
cout<<"LEFT MOTOR :"<< LM<<endl;
cout<<"RIGHT MOTOR :"<<RM<<endl;
}

void calcOdom()
{   
    /*if((device.GetValue(_S, 1, lenc)!=RQ_SUCCESS)||(device.GetValue(_S, 2, renc)!=RQ_SUCCESS))
    {
        ROS_INFO("Encoder data decoding failed\n");
        return;
    }  
    if(device.GetValue(_S, 2, renc)!=RQ_SUCCESS)
    {
        ROS_INFO("Encoder data decoding failed\n");
        return;
    }
    ROS_INFO("ODOM LENC : %d",lenc);
    ROS_INFO("ODOM RENC : %d",lenc);*/
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
    ros::Duration(0.1).sleep();
    ros::Subscriber sub = n.subscribe("joy", 1, teleopCallback);
    ros::Subscriber sub2 = n.subscribe("cmd_vel", 1, cmdVelCallback);
    //ros::Subscriber sub3 =  n.subscribe("amcl_pose", 1, recManCallback);
    ros::Subscriber sub3 = n.subscribe("motor_commands", 1, alignCallback);
    ros::Publisher pub1 = n.advertise<nav_msgs::Path>("sent_path", 50);
    //XARALAMPOS
    ros::Publisher pub2 = n.advertise<std_msgs::Int32MultiArray>("xar_odom", 1000);
    motor_commands.data.push_back(0);
    motor_commands.data.push_back(0);
    //
    //ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    
    ROS_INFO("- SetConfig(_DINA, 1, 1)...");
    if((status = device.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      ROS_INFO("succeeded.");
    ros::Duration(0.01).sleep(); //sleep for 10 ms

    int result;
    
    ROS_INFO("- GetConfig(_DINA, 1)...");
    if((status = device.GetConfig(_DINA, 1, result)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"returned --> "<<result<<endl;
    ROS_INFO("Roboteq -- Successfull setup of the Roboteq SDC2130");
    printf ("Sek Operational\n\n");
    ros::Duration(0.01).sleep(); //sleep for 10 ms
    //current_time = ros::Time::now();
    last_time = ros::Time::now();
    while (ros::ok())
    {   
        ros::spinOnce();
        if (n.hasParam("sek_drive/con_mode"))
        {
            if (n.getParam("sek_drive/con_mode", MODE))
            {
                if (MODE == 1)
                {
                    //cout<<"3SPOOKYSPOOPY5ME"<<endl;
                    //ROS_INFO("3SPOOKYSPOOPY5ME");
                }
                else
                {
                    //ROS_INFO("param != 1");
                }
            }
            else
            {
                //ROS_INFO("could not get param");
            }
        }
        else
        {
            //ROS_INFO("no param named con_mode");
        }
        if (PLAYING==1)
        {
            pub1.publish(path_to_send);
            PLAYING = 0;
        }
        pub2.publish(motor_commands);
    }   
    device.Disconnect();
    return 0;
}

