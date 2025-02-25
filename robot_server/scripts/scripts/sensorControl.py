
import rospy
import roslib
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from robot_server.msg import chatter


class py_to_joy(object):
    #the publisher
    key_pub=None
    #constructor
    def __init__(self,socket,ts=0.25,ms=0.20):
        #orizetai o subscriber poy akoyei sto topic chatter gia enan Integer
        self.chat_sub = rospy.Subscriber('chatter', Int32, self.chatter_cb,queue_size=10)
        #
        #self.chat_sub2 = rospy.Subscriber('chatter_2', chatter, self.chatter_cb2,queue_size=10)
        self.chat_sub2 = rospy.Subscriber('chatter_2', Float64MultiArray, self.chatter_cb2,queue_size=10)
        #orizetai o publisher poy 8a steilei ena minima Joy sto topic joy
        self.joy_pub = rospy.Publisher('joy', Joy)
        #orizetai to minima Joy poy 8a gemisoyme me dedomena
        self.joy_msg = Joy()
        #to minima Joy exei 2 pinakes gia dia8esima plhktra kai axones, to mege8os twn opoion orizoyme
        # me to extend edw estw 2 axones X,Y
        self.joy_msg.axes.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self.joy_msg.buttons.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self.turnSensitivity=ts
        self.moveSensitivity=ms
        self.socket=socket
        
    def chatter_cb(self,data):
        x=data.data
        if(x==1):
            self.joy_msg.axes[6]=1
        elif(x==2):
            self.joy_msg.axes[5]=1
        elif(x==3):
            self.joy_msg.axes[6]=-1
        elif(x==4):
            self.joy_msg.axes[5]=-1
        elif(x==5):#user stops sending data
            self.joy_msg.axes[5]=0
            self.joy_msg.axes[6]=0
        elif(x==0):#user terminates
            self.joy_msg.axes[5]=0
            self.joy_msg.axes[6]=0
        else:
            pass
        self.joy_pub.publish(self.joy_msg)
        self.joy_msg.axes[5]=0
        self.joy_msg.axes[6]=0
    #callback, kaleitai otan er8ei ena minima sto topic chatter
    
    def chatter_cb2(self,data):
        x=data.data
        
        if(x[0]==12.0 or x[1]==12.0):
            self.joy_msg.axes[1]=0.0
            self.joy_msg.axes[0]=0.0
        elif(x[0]==10.0 or x[1]==10.0):
            self.joy_msg.buttons[0]=1
        else:
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

    def controller(self):
        # arxikopoieitai to node
        rospy.init_node('python_to_joy')
        #orizetai o publisher poy 8a stelnei tis entoles
        self.key_pub = rospy.Publisher('chatter', Int32)
        while True:
            try:
                #diabazei thn entolh tou xrhsth apo to socket
                print("W8ting...")
                key=int(self.socket.recv(32))
                #i=threading.activeCount()
                #print i
            except ValueError:
                print("Not an integer")
            #an stal8ei h timh 0 termatizetai to module
            self.key_pub.publish(key)
            print("key ={}".format(key))
            if(key==0):
                break
        return 0
    
    def sensor_controller(self):
        # arxikopoieitai to node
        rospy.init_node('python_to_joy')
        #orizetai o publisher poy 8a stelnei tis entoles
        #self.key_pub = rospy.Publisher('chatter_2', chatter)
        self.key_pub2 = rospy.Publisher('chatter_2', Float64MultiArray)
        #self.socket.sendall(2);
        self.key_msg = Float64MultiArray()
        #oso leitoyrgei to node :
        key_d=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        while True:
            try:
                #diabazei thn entolh tou xrhsth apo to socket
                print("W8ting...")
                self.socket.sendall("1")
                #key[0]=float(self.socket.recv(64))
                #reading front/back values
                key_d[0]=(float(self.socket.recv(64)))
                print("front/back = {}".format(key_d[0]))
                self.socket.sendall("1")
                #reading right/left values
                key_d[1]=float(self.socket.recv(64))
                print("right/left = {}".format(key_d[1]))
            except ValueError:
                print("Not a float")
            self.key_msg.data=key_d
            self.key_pub2.publish(self.key_msg)
            #if value 12.0 is received proccess is ended
            if(key_d[0]==12.0 or key_d[1]==12.0):
                print(12)
                break
            key_d=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            #print("key[0] = {} , key[1] = {}".format(key[0],key[1]))
            
        return 0        
