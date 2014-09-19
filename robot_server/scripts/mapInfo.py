#! /usr/bin/env python
import numpy
import rospy
import roslib
import socket
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from robot_server.msg import chatter
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from nav_msgs.msg import Path
from PIL import Image
import os

class mapInfo(object):
    
    self.debug=True
    def __init__(self,socket):
		#original map
		self.olpMap='/home/skel/ws2/src/robot_server/scripts/mymap.pgm'
		#converted map
		self.newMap='/home/skel/ws2/src/robot_server/scripts/mymap.png'
        #publisher for starting point
        self.pose_pub=rospy.Publisher('initial_pose',PoseWithCovarianceStamped,latch=True)
        #pose message
        self.pose_msg=PoseWithCovarianceStamped()
        #publisher for destination point
        self.pose_pubDest=rospy.Publisher('goal_pose',PoseStamped,latch=True)
        #message to read position
        self.pose_amcl=PoseWithCovarianceStamped()
        #pose message
        self.pose_msgDest=PoseStamped()
        #'Path' message is published in 'sent path'
        self.mult_pub = rospy.Publisher('sent_path',Path)
        #message 'Path' that holds multiple points
        self.mult_msg = Path()
        #publisher that will sent message joy to topic Joy
        self.joy_pub = rospy.Publisher('joy', Joy)
        #messages to be filled with data
        self.joy_msg = Joy()
        self.joy_msg.buttons.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        #matrix with quaternion coordinates
        self.quaternion=[0.0,0.0,0.0,0.0]
        self.socket=socket
        self.covariance = [0] * 36
        self.covariance[0]=0.25
        self.covariance[7]=0.25
        self.covariance[35]=0.06853891945200942

    def garbage(self):
        self.joy_pub.publish(self.joy_msg)
        if(self.debug):
			print("garbage")
        
    def stream(self):
        self.joy_msg.buttons[2]=1
        self.joy_pub.publish(self.joy_msg)
        self.joy_msg.buttons[2]=0
    
    def amcl(self,data):
        x=data.pose.pose.position.x
        y=data.pose.pose.position.y
        self.socket.sendall(str(x))
        self.socket.recv(32)
        self.socket.sendall(str(y))   
             
    def sendMap(self):
        size=self.convertImage()
        image=open(self.oldMap,)
        #image = open('/home/skel/.ros/iit2.png', 'rb')
        imageData=image.read()
        try:
            self.socket.recv(32)
            print("size = {} ".format(size))
            
            self.socket.sendall("{}".format(size))
            self.socket.recv(32)
            print("sending image")
            sent=self.socket.sendall(imageData)
            if(sent==None):
                print("image sent")
            else:
                print("failed to send")
            self.socket.recv(32)
        except socket.error as msg:
            print(msg)
            print("exception")
        pass

    def convertImage(self):
		#if image not already converted
        if ( not os.path.isfile(self.newMap)):
            print("Error: {} file not found".format(self.newMap))
            x=Image.open(self.oldMap)
            imsz=x.size
            print("size = {}".format(imsz))
            newimg=Image.new('L',imsz)
            newimg.putdata(x.getdata())
            newimg.save(self.newMap)
            print("image converted to png format ")
        return os.path.getsize(self.newMap)
    
    def setDestination(self):
        s=self.socket
        try:
            s.sendall('1')
            self.quaternion[0]=(float(s.recv(64)))#receive x
            s.sendall('1')
            self.quaternion[1]=(float(s.recv(64)))#receive y
            s.sendall('1')
            self.quaternion[2]=(float(s.recv(64)))#receive z
            s.sendall('1')
            self.quaternion[3]=(float(s.recv(64)))#receive w
        except ValueError:
            print("Not a float")
            
        self.pose_msgDest.header.frame_id = 'map'
        self.pose_msgDest.pose.position.x=self.quaternion[0]
        self.pose_msgDest.pose.position.y=self.quaternion[1]
        self.pose_msgDest.pose.position.z=0.0
        
        self.pose_msgDest.pose.orientation.x=0.0
        self.pose_msgDest.pose.orientation.y=0.0
        self.pose_msgDest.pose.orientation.z=self.quaternion[2]
        self.pose_msgDest.pose.orientation.w=self.quaternion[3]
        
        self.pose_pubDest.publish(self.pose_msgDest)
        s.sendall('1')
        return 0

    def setMultDestinations(self):
		#read number of points to be sent
		s=self.socket
		s.sendall('1')
		points=(int(s.recv(32)))
		print("for x = {}".format(points))
		for x in range(0 , points):
			#create new Pose item
			print("x={}|".format(x))
			mult_msgDestinations=PoseStamped()
			try:
				s.sendall('1')
				self.quaternion[0]=(float(s.recv(64)))#receive x
				s.sendall('1')
				self.quaternion[1]=(float(s.recv(64)))#receive y
				s.sendall('1')
				self.quaternion[2]=(float(s.recv(64)))#receive z
				s.sendall('1')
				self.quaternion[3]=(float(s.recv(64)))#receive w
			except ValueError:
				print("Not a float")
			mult_msgDestinations.pose.position.x=self.quaternion[0]
			mult_msgDestinations.pose.position.y=self.quaternion[1]
			mult_msgDestinations.pose.position.z=0.0
			
			mult_msgDestinations.pose.orientation.x=0.0
			mult_msgDestinations.pose.orientation.y=0.0
			mult_msgDestinations.pose.orientation.z=self.quaternion[2]
			mult_msgDestinations.pose.orientation.w=self.quaternion[3]
			#add item pose to the list
			self.mult_msg.poses.append(mult_msgDestinations)
			
			self.quaternion=[0.0,0.0,0.0,0.0]
		self.mult_pub.publish(self.mult_msg)
		s.sendall('1')	
		return 0
			
            
    def setStart(self):
        s=self.socket
        try:
            s.sendall('1')
            self.quaternion[0]=(float(s.recv(64)))#receive x
            s.sendall('1')
            self.quaternion[1]=(float(s.recv(64)))#receive y
            s.sendall('1')
            self.quaternion[2]=(float(s.recv(64)))#receive z
            s.sendall('1')
            self.quaternion[3]=(float(s.recv(64)))#receive w
        except ValueError:
            print("Not a float")
        
        self.pose_msg.header.frame_id = 'map'
        self.pose_msg.pose.pose.position.x=self.quaternion[0]
        self.pose_msg.pose.pose.position.y=self.quaternion[1]
        self.pose_msg.pose.pose.position.z=0.0
        
        self.pose_msg.pose.pose.orientation.x=0.0
        self.pose_msg.pose.pose.orientation.y=0.0
        self.pose_msg.pose.pose.orientation.z=self.quaternion[2]
        self.pose_msg.pose.pose.orientation.w=self.quaternion[3]
        if(self.debug):
			print("starting point | x = {} | y = {} ".format(self.quaternion[0],self.quaternion[1]))
        self.pose_msg.pose.covariance=self.covariance
        self.pose_pub.publish(self.pose_msg)
        return 0

        
