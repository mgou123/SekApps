#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#include <stdio.h>
#include <ctype.h>

#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sys/time.h>
#include <chrono>
#include <ctime>

#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv.h"
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

int SCAN_RECEIVED = 0;
int IMAGE_RECEIVED = 0;
double PI =3.141592653589793238;
sensor_msgs::Image gl_image;
sensor_msgs::LaserScan gl_scan;


void merge_stream();
void merge_stream2();
//method that converts radians to degrees
float RadiansToDegrees (float Angle) {
  return Angle * 180.0 / PI;
}

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

//DIABAZEI MIA EIKONA KAI TH GRAFEI STH DOMH POY THA ENWSEIS ME TO SCAN
void imageCallback (const sensor_msgs::Image::ConstPtr& str_img)
{
    if (IMAGE_RECEIVED == 0)
    {
        gl_image.height = str_img->height;
        gl_image.width = str_img->width;
        gl_image.encoding = str_img->encoding;
        gl_image.step = str_img->step;
        gl_image.data.clear();
        gl_image.data = str_img->data;
        IMAGE_RECEIVED = 1;
    }
	if(SCAN_RECEIVED == 1){
		merge_stream();	
	}
}

/*sensor_msgs::ImagePtr*/void merge_stream()
{
    	//our 0,0 here will be the width/2,height of the image coming from kinect
	//so, the points coming from the scanner will be rotated according to angles
	//around this 0,0.
	double startX = gl_image.width/2;
	double startY = gl_image.height;
	float min_angle = /*RadiansToDegrees(*/gl_scan.angle_min;//save the minimum angle in degrees
	float max_angle = /*RadiansToDegrees(*/gl_scan.angle_max;//save the maximum angle in degrees
	float inc = /*RadiansToDegrees(*/gl_scan.angle_increment;		//save the step between angles
	float min_range = gl_scan.range_min;		//save both the min and max values because values
	float max_range = 1;// gl_scan.range_max;		//out of these limits need to be discarded.
	//more info @ http://ftp.isr.ist.utl.pt/pub/roswiki/doc/api/sensor_msgs/html/msg/LaserScan.html
	cv_bridge::CvImagePtr cv_ptr;			//we need to change the format of the image so it's compatible with opencv
	try
	{
		cv_ptr = cv_bridge::toCvCopy(gl_image/*, enc::BGR8*/);
		//Mat cv_img_source = cv_ptr->image;
		float currentAngle = min_angle;
		double endX = -100;
		double endY = -100;
		float measure = gl_image.width / 2 / max_range;//max range will be in the edges of the screen, and the rest will scale according to our image resolution
		for(unsigned i = 0; i < gl_scan.ranges.size(); i++){
			if(gl_scan.ranges[i] >= min_range && gl_scan.ranges[i] <= 1.0){//draw only points within the range limits
				//we move the points from 1st and 4th quadrants, to 2nd and 1st respectively, by adding PI/2 to the degrees
				endX = startX + gl_scan.ranges[i] * measure * cos(currentAngle + (PI/2));
				endY = startY - gl_scan.ranges[i] * measure * sin(currentAngle + (PI/2));			
			}
			circle(cv_ptr->image, cvPoint(endX,endY), /*radius*/4, CV_RGB(0,255,0), -1/*line thickness<0=filled circle*/);
			currentAngle += inc;	
		}
		circle(cv_ptr->image, cvPoint(startX,startY), 10, CV_RGB(255,0,0), -1);	
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	} 
	
    	sensor_msgs::ImagePtr msg = cv_ptr->toImageMsg();
	gl_image.height = msg->height;
        gl_image.width = msg->width;
        gl_image.encoding = msg->encoding;
        gl_image.step = msg->step;
        gl_image.data.clear();
        gl_image.data = msg->data;

}

void merge_stream2(){
	double startX = gl_image.width/2;
	double startY = gl_image.height;
        float min_angle = /*RadiansToDegrees(*/gl_scan.angle_min;//save the minimum angle in degrees
        float max_angle = /*RadiansToDegrees(*/gl_scan.angle_max;//save the maximum angle in degrees
        float inc = /*RadiansToDegrees(*/gl_scan.angle_increment;               //save the step between angles
        float min_range = gl_scan.range_min;            //save both the min and max values because values

	float max_range = 1;// gl_scan.range_max;               //out of these limits need to be discarded.
        //more info @ http://ftp.isr.ist.utl.pt/pub/roswiki/doc/api/sensor_msgs/html/msg/LaserScan.html
        cv_bridge::CvImagePtr cv_ptr;                   //we need to change the format of the image so it's compatible with opencv
        try
        {
                cv_ptr = cv_bridge::toCvCopy(gl_image/*, enc::BGR8*/);
                //Mat cv_img_source = cv_ptr->image;
                float currentAngle = min_angle;
                double endX = -100;
                double endY = -100;
                float measure = gl_image.width / 2 / max_range;//max range will be in the edges of the screen, and the rest will scale according to our image resolution
                for(unsigned i = 0; i < gl_scan.ranges.size(); i++){
                        if(gl_scan.ranges[i] >= min_range && gl_scan.ranges[i] <= 1.0){//draw only points within the range limits
                                //we move the points from 1st and 4th quadrants, to 2nd and 1st respectively, by adding PI/2 to the degrees
                                endX = startX - currentAngle*gl_image.width/2;
                                endY = startY - gl_scan.ranges[i] * measure;
                        }
                        circle(cv_ptr->image, cvPoint(endX,endY), /*radius*/4, CV_RGB(0,255,0), -1/*line thickness<0=filled circle*/);
                        currentAngle += inc;
                }
		line(cv_ptr->image, cvPoint(startX,startY), cvPoint(startX,0), CV_RGB(0,0,0), 2, 8, 0);
		line(cv_ptr->image, cvPoint(0,gl_image.height), cvPoint(gl_image.width,gl_image.height), CV_RGB(0,0,0), 10, 8 ,0);
                circle(cv_ptr->image, cvPoint(startX,startY), 10, CV_RGB(255,0,0), -1);
        }
        catch (cv_bridge::Exception& e)
        {
                ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        sensor_msgs::ImagePtr msg = cv_ptr->toImageMsg();
        gl_image.height = msg->height;
        gl_image.width = msg->width;
        gl_image.encoding = msg->encoding;
        gl_image.step = msg->step;
        gl_image.data.clear();
        gl_image.data = msg->data;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stream_merge");
    ros::NodeHandle n;
    
    //sensor_msgs::Image image;
    sensor_msgs::Image merged_image;
    sensor_msgs::LaserScan laser_scan;
    
    //SUBSCIRBERS - AKOUNE DEDOMENA SE ENA TOPIC (STREAM DEDOMENWN POY ANANEWNETAI PERIODIKA)
    ros::Subscriber img_in;
    ros::Subscriber hok_in;
    hok_in = n.subscribe<sensor_msgs::LaserScan>("scan", 100, scanCallback);
    img_in = n.subscribe<sensor_msgs::Image>("/camera/rgb/image_color", 5, imageCallback);
    
    //PUBLIHSERS - STELNOYN DEDOMENA SE ENA TOPIC
    ros::Publisher merged_img_pub;
     merged_img_pub = n.advertise<sensor_msgs::Image>("/merged_image", 1000);
   
    while (ros::ok())
    {   
        //ROS_INFO("Looping");
        if ((SCAN_RECEIVED == 1) && (IMAGE_RECEIVED==1))
        {   
           //OTAN EXEI KAI EIKONA KAI SCAN KALOYME THN MERGE KAI KANOYME PUBLISH THN EIKONA
           //merged_img_pub.publish(merge(image,laser_scan));
	   merged_img_pub.publish(gl_image);
           SCAN_RECEIVED = 0;
           IMAGE_RECEIVED = 0;
           
        }
        ros::spinOnce();
    }
    return 0;
}


