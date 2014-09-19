
import rospy
import roslib
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
import threading
from threading import Thread


class py_to_joy(object):
    #the publisher
    key_pub=None
    run=False
    #constructor
    def __init__(self,socket):
        #orizetai o subscriber poy akoyei sto topic chatter gia enan Integer
        self.chat_sub = rospy.Subscriber('chatter', Int32, self.chatter_cb2,queue_size=10)
        #orizetai o publisher poy 8a steilei ena minima Joy sto topic joy
        self.joy_pub = rospy.Publisher('joy', Joy)
        #orizetai to minima Joy poy 8a gemisoyme me dedomena
        self.joy_msg = Joy()
        #to minima Joy exei 2 pinakes gia dia8esima plhktra kai axones, to mege8os twn opoion orizoyme
        # me to extend edw estw 2 axones X,Y
        self.joy_msg.axes.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self.joy_msg.buttons.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

        self.socket=socket
        
    def chatter_cb2(self,data):
		x=data.data
		if(x==1):
			self.joy_msg.axes[6]=1
		elif(x==2):
			self.joy_msg.axes[5]=1
		elif(x==3):
			self.joy_msg.axes[6]=-1
		elif(x==4):
			self.joy_msg.axes[5]=-1
		elif(x==5):
			self.joy_msg.axes[5]=0
			self.joy_msg.axes[6]=0
		elif(x==0):
			self.yoy_msg.axes[5]=0
			self.joy_msg.axes[6]=0
			reason="etsi"
			rospy.signal_shutdown(reason)
		else:
			pass
		self.joy_pub.publish(self.joy_msg)
		self.joy_msg.axes[5]=0
		self.joy_msg.axes[6]=0
    #callback, kaleitai otan er8ei ena minima sto topic chatter
    
   # def publish_key(self,key):
   #     #while self.run:
   #     if self.run :
   #         #Endexomenos na steilei ena parapano
	#		self.key_pub.publish(key)
    
    def controller(self):
        # arxikopoieitai to node
        rospy.init_node('python_to_joy')
        #orizetai o publisher poy 8a stelnei tis entoles
        self.key_pub = rospy.Publisher('chatter', Int32)
        t1=None
        #oso leitoyrgei to node :
        while not rospy.is_shutdown():
            try:
				#diabazei thn entolh tou xrhsth apo to socket
				print("W8ting...")
				key=int(self.socket.recv(32))
				#i=threading.activeCount()
				#print i
            except ValueError:
                print("Not an integer")
            #an stal8ei h timh 0 termatizetai to module
            if (key==0):
                reason="User ended manual control"
                print(reason)
                rospy.signal_shutdown(reason)
            elif (key==5):
                self.run=False
                #print(t1.isAlive())
                #t1=None
                #print(threading.activeCount())
                print("stopped")
            elif(key==1 or key==2 or key==3 or key==4 or key==5):
                try:
					#self.run=True
					#t1 = Thread(target=self.publish_key,args=(key,))
					#t1.start()
					#self.publish_key(key)
					self.key_pub.publish(key)
                except:print("Could not start thread")
            else:
                print("Not a valid key {}".format(key))
            print("key ={}".format(key))
            #rospy.spin()
            #o publisher stelnei to minima sto topic chatter gia na to akoysei 
            #o listener poy exei oristei apo panw
                  
    #controller()               
