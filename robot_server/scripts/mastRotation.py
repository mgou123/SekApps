#! /usr/bin/env python
import numpy
import rospy
import roslib
import socket
from std_msgs.msg import Float64
import os

class mastRotation(object):
    
    debug=True
    def __init__(self,socket):
        self.pos_pub = rospy.Publisher("mast_float", Float64, latch = True)
        self.tilt_pub =  rospy.Publisher("tilt_angle", Float64, latch = True)
        self.pos = Float64()
        self.pos.data = 168.0
        self.tilt = Float64()
        self.tilt.data = 0.0
        self.socket=socket
        self.cancel = 0.0
        self.cnt = 0
        self.x = 0.0
        self.y = 0.0
        #self.stream = []
    def rotateMast(self):
       print '1'
       s = self.socket
       s.setblocking(1)
       print '2'
       while True:
           if self.cnt==0 :
              print '3'
              #self.cnt=1
              #s.sendall('1')
              if True :#try:
                  print '4'
                  stream = (str(s.recv(1024))).split()
                  print stream
                  print 'ytrkf'
                  self.pos.data = float(stream[0])
                  print self.pos.data
                  self.tilt.data = float(stream[1])
                  print self.tilt.data
                  self.cancel = float(stream[2])
                  print self.cancel
                  print '5'
                  print "Mast position :" 
                  print self.pos.data==168.0
                  print self.x == 2.0
                  print self.y == 33.0
                  print self.pos.data
                  print self.tilt.data
                  print self.cancel
                  self.pos_pub.publish(self.pos)
                  self.tilt_pub.publish(self.tilt)
                  if (self.cancel == 666):
                      return 0
              if False :#except ValueError:
                  print ("pos data not a float")
                  #break
              
           
       return 0
     #  		self.pos_pub.publish(self.pos)
    #   		self.tilt_pub.publish(self.tilt)
        #self.pos.data = ""
        #return 0
