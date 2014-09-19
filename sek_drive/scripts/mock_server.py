#!/usr/bin/env python

import socket
import sys
import roslib

import rospy
import actionlib
try:
    from sensor_msgs.msg import Joy
except ImportError:
    from joy.msg import Joy

def main():
	print'Started'
	rospy.init_node('mock_server', anonymous=True)
	joy_pub = rospy.Publisher('/joy', Joy)
	joy_msg = Joy()
	for i in range(7):
		joy_msg.axes.append(0.0)
	for j in range(12):
		joy_msg.buttons.append(0)
	joy_msg.axes[6]=1
	#rospy.loginfo('joystick vehicle controller starting')
    
	print 'Setting Host'
	#myHost = '192.168.88.202'
	#myPort = 1234
	myHost = rospy.get_param('/ip', '192.168.88.202')
	myPort = rospy.get_param('/port',8080)
	print "My Host : " + myHost
	print "My Port : " + str(myPort)
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # create a TCP socket
	try:
		s.bind((myHost, myPort))
	except socket.error, msg: 
		print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]# bind it to the server port
	s.listen(5)# allow 5 simultaneous
	# pending connections
	print 'Listening'
	conn, addr=s.accept()
	print 'Accepted'
	print 'Connected with ' + addr[0] + ':' + str(addr[1])
	# wait for next client to connectconnection, address = s.accept() # connection is a new socket
	while True:
		#try:
		#	rospy.spin()
		#except rospy.ROSInterruptException: pass
		print 'in loop'
		data = conn.recv(4) # receive up to 1K bytes
        #print data
		#if not data:
		#	continue
		print data[0]
		if (int(data[0])==2):
			joy_pub.publish(joy_msg)
			print"Received 2"
			continue
			#s.close()
		elif (int(data[0])==7):
			s.close()
			break
	# close socket

if __name__ == '__main__':
    print'oeoeoeo'
    # run main function and exit
    sys.exit(main())


