from sensorControl import py_to_joy
from mapInfo import mapInfo
from mastRotation import mastRotation
import threading
from threading import Thread
import os
import rospy
from RoboskelPos import RoboskelPos

class handler(object):

    run=False
    debug=True
    def __init__(self,socket):
        self.socket=socket
   
    def publish_key(self,key):
        while self.run:
            print(key)
        pass

    def handle(self):
        s=self.socket
        #diabase thn katastash sthn opoia 8elei na metabei o xrhsths
        #success=0
        rospy.init_node('python_to_joy')
        ##################
        x=mapInfo(s)
        x.garbage()#first publish is never published
        del x
        ##################
        z=None
        while(True):
            try:
		state=(str(s.recv(1024))).split()
                #state=int(s.recv(32))
                if(self.debug):
                    print("State = {}".format(state))
            except ValueError:
                    print("Not a valid number at state {}|".format(state))
                    return False
            if (int(state[0])==1):#metabash sthn katastash xeirismou xrhsimopoiontas koumpia
                if(self.debug):
                    print("simple control:State 1")
                x=py_to_joy(s)
                x.controller()
                del x
            elif(int(state[0])==2):#metabash sthn katastash xeirismou xrhsimopoiontas tous ais8hthres
                if(self.debug):
                    print("sensor control:State 2")
                try:
                    #turnSens=float(s.recv(64))
		    turnSens=float(state[1])
		    print("passed here!")
                    #s.sendall('1')
                    #moveSens=float(s.recv(64))
		    moveSens=float(state[2])
	   	    print("and here!")
                except ValueError:
                    print("Not a valid float {} , {}".format(turnSens,moveSens))
                x=py_to_joy(s,turnSens,moveSens)
                x.sensor_controller()
                del x
            elif(int(state[0])==3):#send the map
                if(self.debug):
                    print("send map:State 3")
                x=mapInfo(s)
                x.sendMap()
                del x
            elif(int(state[0])==4):
                if(self.debug):
                    print("set location:State 4")
                x=mapInfo(s)
                x.setStart()#set current location of the robot
                del x
            elif(int(state[0])==5):
                if(self.debug):
                    print("set destination:State 5")
                x=mapInfo(s)
                x.setDestination()#set point to go
                del x
            elif(int(state[0])==6):#cancel automatic navigation
                if(self.debug):
                    print("cancel route:State 6")
                x=py_to_joy(s)
                x.cancelRoute()
                del x
            elif(int(state[0])==7):#monitor robots movements
                if(self.debug):
                    print("movement feedback:state 7")
                if(z==None):
                    z=RoboskelPos(s)
                z.sendPoints()
            elif(int(state[0])==8):#initiate camera steam
                s.sendall("1")
                if(self.debug):
                    print("stream:state 8")
                x=py_to_joy(s)
                x.stream()
            elif(int(state[0])==9):
                if(self.debug):#execute saved pattern
                    print("begin maneuver:State 9")
                x=py_to_joy(s)
                x.beginManeuver()
                s.sendall("1")
            elif(int(state[0])==10):#stop playback
                if(self.debug):
                    print("stop maneuver:State 10")
                x=py_to_joy(s)
                x.stopManeuver()
                s.sendall("1")
            elif(int(state[0])==11):#record a pattern of moves
                if(self.debug):
                    print("begin recording:State 11")
                x=py_to_joy(s)
                x.beginRecording()
            elif(int(state[0])==12):#stop recording pattern
                if(self.debug):
                    print("stop recording:State 12")
                x=py_to_joy(s)
                x.stopRecording()
            elif(int(state[0])==13):#send multiple destination points
                if(self.debug):
                    print("multiple destination points")
                x=mapInfo(s)
                x.setMultDestinations()
            elif(int(state[0])==14):#rotate the mast
                if(self.debug):
                    print("mast rotation")
                x=mastRotation(s)
                x.rotateMast()
            elif(int(state[0])==0):
                print("Ending control")
                return False
            #elif
                #######
                #######
            #elif (state=0):
