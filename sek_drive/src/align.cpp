//#include <iostream>
//#include <fstream>
//#include <sstream>
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

class coffee_manager
{
    protected:
        ros::NodeHandle n_;
        int current_direction_;
        int previous_direction_;
        int ORDER_GIVEN ;
        int ORDER_UNDER_PROCCESS;
        int ORDER_COMPLETED;
        int ORDER_CANCELLED;
        int MARKER_FOUND;
        int SEARCHING;
        int ALIGNING;
        int CUP_FILLED;
        int DESTINATION_REACHED;
        int DESTINATION_SET;
        int RETURNING_ORDER;
        double XY_THRES;
        double YAW_THRES;
        int FORW_SPEED ;
        int TURN_SPEED ;
        double RM, LM;
        double roll, pitch, yaw;
        tf::Quaternion rotation_quat_;
        //target coordinates
        double cof_x_, cof_y_, cof_or_w, cof_or_z;
        
        tf::Quaternion target_quat_, robot_quat_, robot_quat_rotated_;
        ros::Time time1_, time2_ ;
        
        geometry_msgs::Quaternion final_quat;
        geometry_msgs::Quaternion temp_quat_;
        geometry_msgs::PoseWithCovarianceStamped client_spot;
        geometry_msgs::PoseWithCovarianceStamped coffee_spot;
        geometry_msgs::PoseWithCovarianceStamped robot_pose;
        geometry_msgs::PoseWithCovariance marker_pose;

    public:
    
    coffee_manager (ros::NodeHandle& n): n_(n), current_direction_(3), previous_direction_(3),
        ORDER_CANCELLED(0), ORDER_COMPLETED(0), ORDER_GIVEN(0), ORDER_UNDER_PROCCESS(0), MARKER_FOUND(0), SEARCHING(0),
        ALIGNING(0), CUP_FILLED(0), DESTINATION_REACHED(0), DESTINATION_SET(0), RETURNING_ORDER(0), XY_THRES(0.3), YAW_THRES(0.02), FORW_SPEED(0), TURN_SPEED(0), RM(0), LM(0),
        cof_x_(0), cof_y_(0), cof_or_w(0), cof_or_z(0), roll(0), pitch(0), yaw(0)
    {}
    
    bool inPosition()
    {
        ROS_INFO("inPosition");
        ros::Duration(1.0).sleep();
        return (marker_pose.pose.position.x < 0.005) && (marker_pose.pose.position.x > -0.005) && (marker_pose.pose.position.z < 0.3);
    }
    
    void markerCallback(const sek_drive::ARMarker::ConstPtr marker_)
    {
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

    void orderCallback(const std_msgs::Int32::ConstPtr& order)
    {
        ROS_INFO("orderCallback");
        ros::Duration(1.0).sleep();
        ORDER_GIVEN = order->data;
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose_)
    {
        //FOR REAL TIME TESTING the robot calculates its position relatively to the 
        //specified coordinates for the coffee pot
        //if it is close enough it should cancel its current route and start the proccess of 
        //finding the pot
        robot_pose.pose = amcl_pose_->pose;
        robot_pose.header.stamp = ros::Time::now();
        robot_pose.header.frame_id = amcl_pose_->header.frame_id;
        tf::quaternionMsgToTF(amcl_pose_->pose.pose.orientation, robot_quat_);
        /*
        ROS_INFO("ROBOT POSE RECEIVED");
        ROS_INFO("ROBOT X = %f", robot_pos_x_);
        ROS_INFO("ROBOT Y = %f", robot_pos_y_);
        ROS_INFO("ROBOT ORIENTATION X = 0.0");
        ROS_INFO("ROBOT ORIENTATION X = 0.0");
        ROS_INFO("ROBOT ORIENTATION W = %f", robot_or_w_);
        ROS_INFO("ROBOT ORIENTATION Z = %f", robot_or_z_);
        */
        if (DESTINATION_REACHED == 1)
        {
            if (CUP_FILLED == 0);
            {
                system("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID '{}' &"); //cancel its current action
                ALIGNING = 1;
            }
            //else
            //{
                //CUP RELEASE / PERSON IDENTIFICATION
            //}
        }
    }

    void alignCallback(const std_msgs::Int32::ConstPtr& align)
    {
        //FOR testing, let the robot know that it should start rotating
        ALIGNING = align->data;
    }

    void updateParameters(ros::NodeHandle& n_)
    {
        
        n_.getParam("align/xy_thres", XY_THRES);
        n_.getParam("align/yaw_thres", YAW_THRES);
        n_.getParam("align/cof_pos_x", cof_x_);
        n_.getParam("align/cof_pos_y", cof_y_);
        n_.getParam("align/cof_or_w", cof_or_w);
        n_.getParam("align/cof_or_z", cof_or_z);
        n_.getParam("align/marker_found", MARKER_FOUND);
        n_.getParam("align/aligning", ALIGNING);
        n_.getParam("align/searching", SEARCHING);
        n_.getParam("align/order_given", ORDER_GIVEN);
        n_.getParam("align/order_completed", ORDER_COMPLETED);
        n_.getParam("align/order_under_proccess", ORDER_UNDER_PROCCESS);
        n_.getParam("align/order_cancelled", ORDER_CANCELLED);
        n_.getParam("align/cup_filled", CUP_FILLED);
        n_.getParam("align/returning", RETURNING_ORDER);
    }
    
    void checkForOrder(ros::NodeHandle& n_, ros::Publisher& pub)
    {
        ROS_INFO("checkForOrder");
        ros::Duration(1.0).sleep();
        if (n_.getParam("/align/order_given", ORDER_GIVEN))
            {
                if (ORDER_CANCELLED == 1)
                {
                    ROS_INFO("ORDER CANCELLED");
                    ros::Duration(1.0).sleep();
                    //NOTIFY ORDER WAS CANCELLED
                    DESTINATION_SET = 0;
                    DESTINATION_REACHED = 0;
                    ORDER_CANCELLED = 0;
                    ORDER_GIVEN = 0;
                    ORDER_UNDER_PROCCESS = 0;
                    ORDER_COMPLETED = 0;
                    n_.setParam("/align/order_cancelled", 0);
                    n_.setParam("/align/order_given", 0);
                    n_.setParam("/align/order_under_proccess", 0);
                    n_.setParam("/align/order_completed", 0);
                    n_.setParam("/align/destination_set", 0);
                    n_.setParam("/align/destination_reached", 0);
                    if (CUP_FILLED == 1)
                    {
                        //WHAT HAPPENS IF ORDER FAILS WHILE CUP IS FULL ?
                    }
                }
                //else if (ORDER_GIVEN == 0)
                else if (n_.getParam("/align/order_given", ORDER_GIVEN) || n_.getParam("/align/cup_filled", CUP_FILLED))
                {
                    ROS_INFO("RETURNING ORDER GIVEN : %d", ORDER_GIVEN);
                    ROS_INFO("RETURNING ORDER CANCELLED : %d", ORDER_CANCELLED);
                    ros::Duration(1.0).sleep();
                    if ((ORDER_GIVEN == 1) && (CUP_FILLED == 0))
                    {   
                        ROS_INFO("order given, cup not full");
                        
                        ros::Duration(1.0).sleep();
                        client_spot.pose = robot_pose.pose;
                        client_spot.header.frame_id = robot_pose.header.frame_id;
                        //set coffe machine as destination
                        if(n_.getParam("/align/destination_set", DESTINATION_SET))
                        {
                            if(DESTINATION_SET == 0)
                            {
                                coffee_spot.header.stamp = ros::Time::now();
                                pub.publish(coffee_spot);
                                DESTINATION_SET = 1;
                                n_.setParam("/align/destination_set", 1);
                                ROS_INFO("destination set, moving to coffee pot");
                            }
                        }
                    }
                    /*
                    else if ((ORDER_GIVEN == 1)&&(CUP_FILLED==1))
                    {
                        ROS_INFO("order not given");
                        ros::Duration(1.0).sleep();
                    }
                    */
                }
                
            }
    }
    
    void fillCup(ros::NodeHandle& n_)
    {
        int sock, lele, tmp;
        struct sockaddr_in server; 
        struct hostent *hp;
        char buff[256];
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock <0) {
            perror("SOCKET FAILED");
            return ;
        }
        server.sin_family=AF_INET;
        if(inet_pton(AF_INET, "192.168.88.230", &server.sin_addr)<=0)
        {
            printf("\n inet_pton error occured\n");
            return ;
        }
        server.sin_port=htons(8081);
        if (connect(sock, (struct sockaddr *) &server, sizeof(server)) < 0) {
            perror("connect failed");
            close(sock);
            return ;
            }
        while (1) 
        {
            //printf("Please enter the message: ");
            //bzero(buff,256);
            //fgets(buff,255,stdin);
            cout <<endl << "about to send 1" << endl;
            string data = "0";
            lele = write(sock, data.c_str(), strlen(data.c_str()));
            if (lele < 0) 
            {
                perror("ERROR writing to socket");
                return ;
            }
            bzero(buff,1);
            cout <<"reading" << endl;
            lele = read(sock, buff, 256);
            cout << "read :" << buff << endl;
            if (lele < 0)
            {
                perror("ERROR reading from socket");
                return ;
            }
            printf(" Got %s /n", buff);
            if(string(buff)=="theend")
            {
                cout << "CUP FILLED" << endl;
                close(sock);
                return ;
            }
            else
            {
                printf("Gor Wrong Signal /n");
                return ;
            }
        } 
        //printf("Sent %s\n", buff);
        close(sock);
    }
    
    void updateMarkerFound(ros::NodeHandle& n_)
    {
        ROS_INFO("updateMarkerFound");
        ros::Duration(1.0).sleep();
        n_.getParam("/align/marker_found", MARKER_FOUND);
        return;
    }
    
    void alignWithMarker(ros::NodeHandle& n_, ros::Publisher& motorPub, ros::Publisher& destPub)
    {
        std_msgs::Int32MultiArray motor_commands;
        motor_commands.data.clear();;
        //ros::Duration(3.0).sleep();
        //CLIENT SERVER WAIT UNTIL CUP IS FULL
        //server sends 0 when cup is full
        if (ALIGNING==1)
        {   
            ROS_INFO("ALIGNING");
            if (MARKER_FOUND == 1 )
            {   
                ROS_INFO("MARKER FOUND");
                if (RETURNING_ORDER == 0)
                {
                    //We have found the marker, the robot should stop and align
                    //only if it is not returning to the client
                    if(DESTINATION_REACHED == 0)
                    {
                         DESTINATION_REACHED = 1;
                        system("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID '{}' &");
                    }
                }
                //if(inPosition() && CUP_FILLED==0)
                if ( (inPosition()||(DESTINATION_REACHED==1)) && CUP_FILLED==0)
                {
                    motor_commands.data.push_back(0);
                    motor_commands.data.push_back(0);
                    motorPub.publish(motor_commands);
                    motor_commands.data.clear();
                    ROS_INFO("FILLING CUP");
                    //fillCup(n_);
                    ROS_INFO("Cup full, returning to client");
                    ros::Duration(1.0).sleep();
                    return;
                    // The robot has completed the interaction with the valve
                    // it should head back to the client
                    
                    client_spot.header.stamp = ros::Time::now();
                    destPub.publish(client_spot);
                    DESTINATION_SET = 1;
                    RETURNING_ORDER = 1;
                    n_.setParam("align/destination_set", DESTINATION_SET);
                    n_.setParam("align/returning", RETURNING_ORDER);
                }
                else
                {
                    if (false)
                    {}
                    //check if the marker has left our f.o.v. for a long time
                    /*
                    if ((ros::Time::now().toSec() - time1_.toSec()) > 0.1) 
                    {
                        MARKER_FOUND = 0;
                        previous_direction_ = current_direction_;
                        current_direction_ = 2;
                        motor_commands.data.push_back(0);
                        motor_commands.data.push_back(0);
                        motorPub.publish(motor_commands);
                        motor_commands.data.clear();
                        ROS_INFO("MARKER LOST");
                    }
                    */
                    else
                    {
                        if ((marker_pose.pose.position.x < 0.02) && (marker_pose.pose.position.x > -0.02))
                        //if ((marker_pos_x_ < 0.005) && (marker_pos_x_ > -0.005))
                        {
                            if (previous_direction_==3 && current_direction_==3)
                            {
                                ROS_INFO("CENTERED AR MARKER");
                                previous_direction_ = 0 ;
                                current_direction_ = 0;
                                motor_commands.data.push_back(-FORW_SPEED);
                                motor_commands.data.push_back(FORW_SPEED);
                                ROS_INFO("MOVING FORWARD");
                                motorPub.publish(motor_commands);
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
                                    motor_commands.data.push_back(-FORW_SPEED);
                                    motor_commands.data.push_back(FORW_SPEED);
                                    ROS_INFO("MOVING FORWARD");
                                    motorPub.publish(motor_commands);
                                    motor_commands.data.clear();
                                }
                                else if (marker_pose.pose.position.z <= 0.40)
                                {
                                    ROS_INFO("IN POSITION");
                                    motor_commands.data.push_back(0);
                                    motor_commands.data.push_back(0);
                                    motorPub.publish(motor_commands);
                                    motor_commands.data.clear();
                                    ros::Duration(10.0).sleep();
                                    return;
                                }
                            }
                        }
                        else if (marker_pose.pose.position.x < - YAW_THRES)
                        //else if (marker_pos_x_ < -0.005) //marker is at the right side of the camera
                        {
                            current_direction_ = -1;
                            if (current_direction_ != previous_direction_)
                            {
                                previous_direction_ = current_direction_ ;
                                motor_commands.data.push_back(-TURN_SPEED);
                                motor_commands.data.push_back(-TURN_SPEED);
                                ROS_INFO("ROTATING LEFT TO ALIGN");
                                motorPub.publish(motor_commands);
                                motor_commands.data.clear();
                            }
                        }
                        else if (marker_pose.pose.position.x > YAW_THRES)
                        //else if (marker_pos_x_ > 0.005) //marker is at the left of the camera
                        {
                            current_direction_ = 1;
                            if (current_direction_ != previous_direction_)
                            {
                                previous_direction_ = current_direction_ ;
                                //current_direction_ = 1;
                                motor_commands.data.push_back(TURN_SPEED);
                                motor_commands.data.push_back(TURN_SPEED);
                                ROS_INFO("ROTATING RIGHT TO ALIGN");
                                motorPub.publish(motor_commands);
                                motor_commands.data.clear();
                            }
                        }
                    }
                }
            }
            else if (SEARCHING == 0)
            {
                ROS_INFO("SEARCHING");
                //AT THIS POINT the robot has reached the desired position 
                // and it may or may not be facing the marker
                // based on its estimated position and the given position of the pot it must
                //caclulate the direction for the shortest turn 
                robot_quat_rotated_ = target_quat_ * robot_quat_.inverse();
                tf::Matrix3x3(robot_quat_rotated_).getRPY(roll, pitch, yaw);
                if (yaw < 0)
                {
                    //rotate left
                    robot_quat_rotated_.setZ(-robot_quat_rotated_.getZ());
                    ROS_INFO("YAW = %f", yaw);
                    //ros::Duration(5.0).sleep();
                    motor_commands.data.push_back(-80);
                    motor_commands.data.push_back(-80);
                    ROS_INFO("SEARCHING FOR MARKER");
                    motorPub.publish(motor_commands);
                    motor_commands.data.clear();
                }
                else
                {   
                    //rotate right
                    ROS_INFO("YAW = %f", yaw);
                    //ros::Duration(5.0).sleep();
                    motor_commands.data.push_back(80);
                    motor_commands.data.push_back(80);
                    ROS_INFO("SEARCHING FOR MARKER");
                    motorPub.publish(motor_commands);
                    motor_commands.data.clear();
                }
                ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
                SEARCHING = 1;
            }
        }
    }
    
    void checkDelivery(ros::NodeHandle& n_)
    {
        ROS_INFO("checkDelivery");
        ROS_INFO("RETURNING ORDER : %d", RETURNING_ORDER);
        ROS_INFO("DESTINATION REACHED: %d", DESTINATION_REACHED);
        if ((n_.getParam("/align/returning", RETURNING_ORDER)) && (n_.getParam("/align/destination_reached", DESTINATION_REACHED)))
        {
            if ((RETURNING_ORDER==1) && (DESTINATION_REACHED == 1))
            {
                n_.setParam("/check_for_client",0);
                ROS_INFO("ORDER SUCCESSFULLY DELIVERED, WAITING FOR NEW ORDER");
                ros::Duration(5.0).sleep();
                DESTINATION_SET = 0;
                DESTINATION_REACHED = 0;
                ORDER_CANCELLED = 0;
                ORDER_GIVEN = 0;
                ORDER_UNDER_PROCCESS = 0;
                ORDER_COMPLETED = 0;
                CUP_FILLED = 0;
                RETURNING_ORDER = 0;
                n_.setParam("/align/order_cancelled", 0);
                n_.setParam("/align/order_given", 0);
                n_.setParam("/align/order_under_proccess", 0);
                n_.setParam("/align/order_completed", 0);
                n_.setParam("/align/destination_set", 0);
                n_.setParam("/align/destination_reached", 0);
                n_.setParam("/align/cup_filled", 0);
                n_.setParam("/align/returning", 0);
            }
        }
    }
    
    void spin()
    {
        ros::Subscriber sub_ = n_.subscribe("ar_pose_marker", 1, &coffee_manager::markerCallback, this);
        ros::Subscriber sub2_ = n_.subscribe("align", 1, &coffee_manager::alignCallback, this);
        ros::Subscriber sub3_ =  n_.subscribe("amcl_pose", 1, &coffee_manager::poseCallback, this);
        //ros::Subscriber sun4_ =  n.subscribe("test_pose", 1, testPoseCallback);
        ros::Subscriber sub5_ = n_.subscribe("order_given", 1, &coffee_manager::orderCallback, this);
        ros::Publisher pub1_ = n_.advertise<std_msgs::Int32MultiArray>("motor_commands", 50);
        ros::Publisher pub2_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("current_robot_destination", 50);
        //change the coffee pot location from the parameter server
        //or launch file
        //Initialize Parameters
        updateParameters(n_);
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
        ROS_INFO("STARTING ALIGNMENT");
        cout<<"Starting Alignment"<<endl;
        ros::Duration(0.01).sleep();
        ROS_INFO("STARTING ORDER_GIVEN : %d", ORDER_GIVEN);
        //client for connecting to valve
        while(ros::ok())
        {
            ros::spinOnce();
            updateMarkerFound(n_);
            updateParameters(n_);
            checkForOrder(n_, pub2_);
            //ros::Duration(3.0).sleep();
            //PART FOR ALIGNING WITH MARKER
            alignWithMarker(n_, pub1_, pub2_);
            fillCup(n_);
            return ;
            checkDelivery(n_);
        }
    }
    ~coffee_manager() {}
} ;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "align");
    ros::NodeHandle nh;
    coffee_manager* cm = 0;
    cm = new coffee_manager(nh);
    cm->spin();
    return 0;
}
