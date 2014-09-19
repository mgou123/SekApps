#!/usr/bin/python

import roslib

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import radians
from nav_msgs.msg import Path
import thread

class RoboskelPos():
    def __init__(self,socket):
        self.socket=socket
        self.set_=0
        #Goal state return values
        self.x=0.0
        self.y=0.0
        self.run=True
        self.path_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.pathCallback)
        try:
            thread.start_new_thread(  (self.PosThread,"Thread-1", 2, ) )
        except:
            print("Error: unable to start thread")
    
    def pathCallback(self,data):
        self.x=data.pose.pose.position.x
        self.y=data.pose.pose.position.y
        self.set_=1
    
    def PosThread(self,threadName, delay):
        while not rospy.is_shutdown():
            if (self.set_==1 and self.run):
                rospy.loginfo(self.x)
                rospy.loginfo(self.y)
                self.set_=0
        
    def sendPoints(self):
        self.socket.sendall('1')
        read=int(self.socket.recv(32))
        self.socket.sendall(str(self.y))
        print("y={}".format(self.y))
        print("x={}".format(self.x))
        read1=int(self.socket.recv(32))
        self.socket.sendall(str(self.x))
