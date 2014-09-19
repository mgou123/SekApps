import rospy
import roslib
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from robot_server.msg import chatter


class py_to_joy(object):
    
    key_pub=None
    debug=True
    #constructor
    def __init__(self,socket,ts=0.25,ms=0.20):
        #subscriber that listens to topic 'chatter' for an integer
        self.chat_sub = rospy.Subscriber('chatter', Int32, self.chatter_cb,queue_size=10)
        #subscriber that listens to topic 'chatter' for floats
        self.chat_sub2 = rospy.Subscriber('chatter_2', Float64MultiArray, self.chatter_cb2,queue_size=10)
        #publisher that will sent message joy to topic Joy
        self.joy_pub = rospy.Publisher('joy', Joy)
        #message to be filled with data
        self.joy_msg = Joy()
        #message Joy contains 2 matrices for axes and buttons
        self.joy_msg.axes.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self.joy_msg.buttons.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self.turnSensitivity=ts
        self.moveSensitivity=ms
        self.socket=socket
        
    def cancelRoute(self):
        self.joy_msg.buttons[1]=1.0
        self.joy_pub.publish(self.joy_msg)
        self.joy_msg.buttons[1]=0.0
        
    def beginRecording(self):
        self.joy_msg.buttons[6]=1.0
        self.joy_pub.publish(self.joy_msg)
        self.joy_msg.buttons[6]=0.0
        
    def stopRecording(self):
        self.joy_msg.buttons[7]=1.0
        self.joy_pub.publish(self.joy_msg)
        self.joy_msg.buttons[7]=0.0
        
    def beginManeuver(self):
        self.joy_msg.buttons[4]=1.0
        self.joy_pub.publish(self.joy_msg)
        self.joy_msg.buttons[4]=0.0
        
    def stopManeuver(self):
        self.joy_msg.buttons[5]=1.0
        self.joy_pub.publish(self.joy_msg)
        self.joy_msg.buttons[5]=0.0
        
	def saveMap(self):
		self.joy_msg.buttons[3]=1
		self.joy_pub.publish(self.joy_msg)
		self.joy_msg.buttons[3]=0
		
    def stream(self):
        self.joy_msg.buttons[2]=1.0
        self.joy_pub.publish(self.joy_msg)
        self.joy_msg.buttons[2]=0.0
            
    def chatter_cb(self,data):
        x=data.data
        if(x==1 or x==11):#sometimes move value is send twice
            self.joy_msg.axes[6]=1
        elif(x==2 or x==22):
            self.joy_msg.axes[5]=1
        elif(x==3 or x==33):
            self.joy_msg.axes[6]=-1
        elif(x==4 or x==44):
            self.joy_msg.axes[5]=-1
        elif(x==5 or x==55):#user stops sending data
            self.joy_msg.axes[5]=0
            self.joy_msg.axes[6]=0
        elif(x==0):#user terminates
            self.joy_msg.axes[5]=0
            self.joy_msg.axes[6]=0
        elif(x==14):#save the map
            self.saveMap()
        elif(x==8):#begin streaming(if user decides to stream while controlling)
            self.stream()
        else:
            pass
        self.joy_pub.publish(self.joy_msg)
        self.joy_msg.axes[5]=0
        self.joy_msg.axes[6]=0
    
    def chatter_cb2(self,data):
        x=data.data
        
        if(x[0]==13.0 or x[1]==13.0):#Stop moving when 13 is received
            self.joy_msg.axes[1]=0.0
            self.joy_msg.axes[0]=0.0
        elif(x[0]==14):#Save the map when 14 is received
            self.joy_msg.buttons[3]=1
        elif(x[0]==15.0 or x[1]==15.0):#Horn
            self.joy_msg.buttons[0]=1
        else:#movement sensitivity to avoid bad control(e.g. robot starts moving at 0.2 instead of 0.0)
            if(x[0]>=-1.0-self.moveSensitivity and x[0]<=1.0+self.moveSensitivity):
                if(x[0]>-self.moveSensitivity and x[0]<self.moveSensitivity):
                    self.joy_msg.axes[1]=0.0#front/back
                else:
                    if(x[0]>0):
                        self.joy_msg.axes[1]=x[0]-self.moveSensitivity
                    else:
                        self.joy_msg.axes[1]=x[0]+self.moveSensitivity#front/back
            else:
                if(x[0]>0):
                    self.joy_msg.axes[1]=1.0
                elif(x[0]<0):   
                    self.joy_msg.axes[1]=-1.0
            
            if(x[1]>=-1.0-self.turnSensitivity and x[1]<=1.0+self.turnSensitivity):
                if(x[1]>-self.turnSensitivity and x[1]<self.turnSensitivity):
                    self.joy_msg.axes[0]=0.0#right/left
                else:
                    if(x[1]>0):
                        self.joy_msg.axes[0]=x[1]-self.moveSensitivity
                    else:
                        self.joy_msg.axes[0]=x[1]+self.moveSensitivity#right/left
            else:
                if(x[1]>0):
                    self.joy_msg.axes[0]=1.0
                elif(x[1]<0):
                    self.joy_msg.axes[0]=-1.0
        
        self.joy_pub.publish(self.joy_msg)
        self.joy_msg.buttons[0]=0.0
        self.joy_msg.axes[0]=0.0
        self.joy_msg.axes[1]=0.0
        self.joy_msg.buttons[3]=0

    def controller(self):
        self.key_pub = rospy.Publisher('chatter', Int32)
        while True:
            try:
                #reads user command
                print("W8ting...")
                key=int(self.socket.recv(32))
            except ValueError:
                print("Not an integer")
            #if value zero is sent module exits
            self.key_pub.publish(key)
            print("key ={}".format(key))
            if(key==0):
                break
        return 0
    
    def sensor_controller(self):
        #publisher that sends moving commands
        self.key_pub2 = rospy.Publisher('chatter_2', Float64MultiArray)
        self.key_msg = Float64MultiArray()
        key_d=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        while True:
            try:
                #reads user command
                print("W8ting...")
                self.socket.sendall("1")
                #reading front/back values
                key_d[0]=(float(self.socket.recv(64)))
                self.socket.sendall("1")
                if(key_d[0]>=8.0 and key_d[0]<=12.0):
                    if(key_d[0]==8):#begin streaming
						if(self.debug):
							print("stream")
						self.stream()
                    elif(key_d[0]==9):
                        self.beginManeuver()
                        if(self.debug):
							print("begin Maneuver")
                    elif(key_d[0]==10):
                        self.stopManeuver()
                        if(self.debug):
							print("stop maneuver")
                    elif(key_d[0]==11):
                        self.beginRecording()
                        if(self.debug):
							print("begin recording")
                    elif(key_d[0]==12):
                        self.stopRecording()
                        if(self.debug):
							print("stop recording")
                else:
					print("front/back = {}".format(key_d[0]))
					#reading right/left values
					key_d[1]=float(self.socket.recv(64))
					print("right/left = {}".format(key_d[1]))
            except ValueError:
                print("Not a float")
            self.key_msg.data=key_d
            if(not(key_d[0]>=8 and key_d[0]<=12)):
                self.key_pub2.publish(self.key_msg)
            #if value 13.0 is received proccess is ended
            if(key_d[0]==13.0 or key_d[1]==13.0):
                print("Exiting Control")
                break
            key_d=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        return 0        
