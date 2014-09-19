/*
 * servo_mast/main.cpp
 *
 * Description:
 * ROS node for controlling servo motor.
 * This node is used for controlling the servo that turns the mast
 * where our robot's Kinect and mic are mounted, but should work for
 * any servo controlled through the Phidget library.
 *
 * Created on: 6 Mar 2013
 * Author: Stasinos Konstantopoulos <stasinos@konstant.gr>
 * Copyright (C) 2013 Stasinos Konstantopoulos <stasinos@konstant.gr>
 */

#include <signal.h>
#include <stdio.h>
#include <phidget21.h>

#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <std_msgs/Float64.h>

#include "servo_mast/mast_position.h"
#include "servo_mast/mast_turn.h"


/*
 * Globals rule
 */


ros::Publisher publisher;
ros::Publisher publisher_2;
// Declare an advanced servo handle
CPhidgetAdvancedServoHandle servo = 0;
double minAcc, maxAcc, minVel, maxVel;

double serv_pos;
double current_pos;

/*
 * Graceful shutdown.
 * If you don't do this, the motor will "lock" if you kill
 * the process, and you will have to power-cycle the robot.
 *
 * Used this:
 * http://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
 * to get shutdownHandler() to execute at shutdown
 */


// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler( int sig )
{
	g_request_shutdown = 1;
}


// Replacement "shutdown" XMLRPC callback
void shutdownCallback( XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result )
{
	if( (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
			&& (params.size() > 1) ) {
		std::string reason = params[1];
		ROS_WARN( "Shutdown request received. Reason: [%s]", reason.c_str() );
		g_request_shutdown = 1;
	}
	result = ros::xmlrpc::responseInt( 1, "", 0 );
}


void shutdownStuff()
{
	CPhidgetAdvancedServo_setEngaged( servo, 0, 0 );
	CPhidget_close( (CPhidgetHandle)servo );
	CPhidget_delete( (CPhidgetHandle)servo );
}



/*
 * Handlers
 */


int CCONV AttachHandler( CPhidgetHandle ADVSERVO, void *userptr )
{
	int serialNo;
	const char *name;
	CPhidget_getDeviceName( ADVSERVO, &name );
	CPhidget_getSerialNumber( ADVSERVO, &serialNo );
	printf( "%s %10d attached!\n", name, serialNo );
	return 0;
}

int CCONV DetachHandler( CPhidgetHandle ADVSERVO, void *userptr )
{
	int serialNo;
	const char *name;
	CPhidget_getDeviceName( ADVSERVO, &name );
	CPhidget_getSerialNumber( ADVSERVO, &serialNo );
	printf( "%s %10d detached!\n", name, serialNo );
	return 0;
}

int CCONV ErrorHandler( CPhidgetHandle ADVSERVO, void *userptr, int ErrorCode, const char *Description )
{
	printf("Error handled. %d - %s\n", ErrorCode, Description);
	return 0;
}

int CCONV PositionChangeHandler( CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value )
{
	servo_mast::mast_position msg;
	msg.position = Value;
	publisher_2.publish( msg );
	return 0;
}



void turnCallback( const servo_mast::mast_turn::ConstPtr& msg )
{
	//change the motor position
	//valid range is -23 to 232, but for most motors ~30-210

	double pos = msg->position;
	double acc = msg->acceleration;
	double vel = msg->velocity;
	bool engage = true;
	if( (pos > 30.0) && (pos < 210.0) ) {
		CPhidgetAdvancedServo_setPosition( servo, 0, pos );
	}
	else { engage = false; }
	if( engage && (acc > minAcc) && (acc < maxAcc) ) {
		CPhidgetAdvancedServo_setAcceleration( servo, 0, acc );
	}
	else { engage = false; }
	if( engage && (vel > minVel) && (vel < maxVel) ) {
		CPhidgetAdvancedServo_setVelocityLimit( servo, 0, vel );
	}
	else { engage = false; }

	if( engage ) {
		CPhidgetAdvancedServo_setEngaged( servo, 0, 1 );
	}
}

void floatCallback( const std_msgs::Float64::ConstPtr& msg )
{
	double pos;
	pos = (double)msg->data;
	serv_pos=pos;
	printf( "Got %f \n", pos );;
	if( (pos > -30.0) && (pos < 210.0) ) {
		CPhidgetAdvancedServo_setPosition( servo, 0, pos );
		CPhidgetAdvancedServo_setEngaged( servo, 0, 1 );
	}
}


/*
 * Misc
 */

// Display the properties of the attached phidget to the screen.

void display_properties( CPhidgetAdvancedServoHandle phid )
{
	int serialNo, version, numMotors;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetAdvancedServo_getMotorCount (phid, &numMotors);

	printf("%s\n", ptr);

	printf("Serial Number: %10d\nVersion: %8d\n# Motors: %d\n", serialNo, version, numMotors);

	printf( "Parameters:\nmin acceleration: %f\nmax acceleration: %f\nmin velocity: %f\nmax velocity: %f\n",
			minAcc, maxAcc, minVel, maxVel );
}


int main(int argc, char* argv[])
{
	int result;
	const char *err;
	double position;
	// Init ROS node overriding SIGINT (roslaunch, ctrl-c)
	// and XMLRPC shutdown (rosnode kill)
	// See also ticket
	// https://code.ros.org/trac/ros/ticket/3417
	// as a unified solution might appear in future ROS versions

	ros::init( argc, argv, "servo_mast", ros::init_options::NoSigintHandler );
	signal( SIGINT, mySigIntHandler );
	// Override  shutdown
	ros::XMLRPCManager::instance()->unbind( "shutdown" );
	ros::XMLRPCManager::instance()->bind( "shutdown", shutdownCallback );

	ros::NodeHandle nodeHandle;

	//create the advanced servo object
	CPhidgetAdvancedServo_create( &servo );

	// Set the handlers to be run when the device is plugged in or opened from software,
	// unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler( (CPhidgetHandle)servo, AttachHandler, NULL );
	CPhidget_set_OnDetach_Handler( (CPhidgetHandle)servo, DetachHandler, NULL );
	CPhidget_set_OnError_Handler( (CPhidgetHandle)servo, ErrorHandler, NULL );

	// Registers a callback that will run when the motor position is changed.
	// Args:
	//  the handle for the Phidget
	//  the function that will be called
	//  arbitrary pointer that will be supplied to the callback function (may be NULL)
	CPhidgetAdvancedServo_set_OnPositionChange_Handler( servo, PositionChangeHandler, NULL );

	//open the device for connections
	CPhidget_open( (CPhidgetHandle)servo, -1 );

	//get the program to wait for an advanced servo device to be attached
	printf("Waiting for Phidget to be attached....");
	result = CPhidget_waitForAttachment( (CPhidgetHandle)servo, 10000 );
	if( result ) {
		CPhidget_getErrorDescription( result, &err );
		printf( "Problem attaching Phidget: %s\n", err );
		return 0;
	}
	else {
	}

	CPhidgetAdvancedServo_getAccelerationMin(servo, 0, &minAcc);
	CPhidgetAdvancedServo_getAccelerationMax(servo, 0, &maxAcc);
	CPhidgetAdvancedServo_getVelocityMin(servo, 0, &minVel);
	CPhidgetAdvancedServo_getVelocityMax(servo, 0, &maxVel);
    maxVel = maxVel/320;
    maxAcc = maxAcc/12800;
	CPhidgetAdvancedServo_setAcceleration( servo, 0, maxAcc);
	CPhidgetAdvancedServo_setVelocityLimit( servo, 0, maxVel);
    
	display_properties( servo );
    
	// Defaults. If user only publishes float64 messages, these will be used
	//CPhidgetAdvancedServo_setAcceleration( servo, 0, minAcc*2/4 );
	//CPhidgetAdvancedServo_setVelocityLimit( servo, 0, maxVel/8 );

    publisher = nodeHandle.advertise<std_msgs::Float64>( "mast_float2", 100 );
    publisher_2 = nodeHandle.advertise<servo_mast::mast_position>("mast_position", 100);
    ros::Subscriber subscriber1 = nodeHandle.subscribe( "mast_turn", 1000, turnCallback );
    //ros::Subscriber subscriber2 = nodeHandle.subscribe( "mast_float", 1000, floatCallback );
    ros::Subscriber subscriber2 = nodeHandle.subscribe( "mast_float", 100, floatCallback );

    while( !g_request_shutdown ) {
    	std_msgs::Float64 msg;
    	msg.data=serv_pos;
    	//publisher.publish(msg);
    	ros::spinOnce();
        //usleep(10000);
        //CPhidgetAdvancedServo_setEngaged( servo, 0, 1 );
        //CPhidgetAdvancedServo_setPosition( servo, 0, 180 );
        //printf("Motor: 0 > Current Position: %f\n", current_pos);
    	//usleep(100000);
    	usleep(3000);

    }

    printf( "Closing...\n" );
    shutdownStuff();

    return 0;
}
