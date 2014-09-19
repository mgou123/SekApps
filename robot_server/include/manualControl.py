
import rospy
import roslib
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
import threading


class py_to_joy(object):
    #the publisher
    key_pub=None
    run=False
    #constructor
    def __init__(self,socket):
        #orizetai o subscriber poy akoyei sto topic chatter gia enan Integer
        self.chat_sub = rospy.Subscriber('chatter', Int32, self.chatter_cb2)
        #orizetai o publisher poy 8a steilei ena minima Joy sto topic joy
        self.joy_pub = rospy.Publisher('joy', Joy)
        #orizetai to minima Joy poy 8a gemisoyme me dedomena
        self.joy_msg = Joy()
        #to minima Joy exei 2 pinakes gia dia8esima plhktra kai axones, to mege8os twn opoion orizoyme
        # me to extend edw estw 2 axones X,Y
        self.joy_msg.axes.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self.joy_msg.buttons.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

        self.socket=socket
        
    
    #callback, kaleitai otan er8ei ena minima sto topic chatter
    def chatter_cb2(self,data):
        #ta dedomena toy minimatos sto X
        x=data.data;
        #analoga me tis times toy X allazoyn oi times ton axonwn
        if (x==1):
            #up
            self.joy_msg.axes[6]=1
            print("1:Go straight")
        elif(x==2):
            #left
            self.joy_msg.axes[5]=1
            print("2:go left")
        elif(x==3):
            #back
            self.joy_msg.axes[6]=-1
            print("3:go back")
        elif(x==4):
            #right
            self.joy_msg.axes[5]=-1
            print("4:go right")
        else:
            pass
        # o publisher stelnei to minima sto topic joy
        # kai oi axones xanaarxikopoioyntai 
        self.joy_pub.publish(self.joy_msg)
        self.joy_msg.axes[0]=0
        self.joy_msg.axes[1]=0
        
    def publish_key(self,key):
        while self.run:
            #Endexomenos na steilei ena parapano
            self.key_pub.publish(key)
        pass
    
    def controller(self):
        # arxikopoieitai to node
        rospy.init_node('python_to_joy')
        #orizetai o publisher poy 8a stelnei tis entoles
        self.key_pub = rospy.Publisher('chatter', Int32)
        py_to_joy()
        t1=None
        #oso leitoyrgei to node :
        while not rospy.is_shutdown():
            try:
                #diabazei thn entolh tou xrhsth apo to socket
                key=int(self.socket.recv(32))
            except ValueError:
                print("Not an integer")
            #an stal8ei h timh 0 termatizetai to module
            if (key==0):
                reason="User ended manual control"
                print(reason)
                rospy.signal_shutdown(reason)
            elif(key==5):
                self.run=False
                t1._stop()
                print(t1.isAlive())
                t1=None
                print("stopped")
            else:
                try:
                    self.run=True
                    t1 = threading.Thread(target=self.publish_key(key))
                    t1.start()
                except:print("Could not start thread")
            print("key ={}".format(key))
            
            #o publisher stelnei to minima sto topic chatter gia na to akoysei 
            #o listener poy exei oristei apo panw
                  
    #controller()               