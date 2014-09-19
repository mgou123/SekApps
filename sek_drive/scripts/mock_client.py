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
    
def joyCallback(joy):
    "invoked every time a joystick message arrives"
    rospy.logdebug('joystick input:\n' + str(joy))
    if(len(joy.buttons)==0):
        print 'faulty input'
        return
    #print joy
    #server_address=(self.myHost, self.myPort)
    if(joy.buttons[0]==1):
        # Create a TCP/IP socket
        #sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		# Connect the socket to the port where the server is listening
        #server_address = ('192.168.88.202', 1234)
        server_address = ('192.168.88.222', 1234)
        #server_address=(myHost, myPort)
        print >>sys.stderr, 'connecting to %s port %s' % server_address
        sock.connect(server_address)
        message = '1'
        print >>sys.stderr, 'sending "%s"' % message
        #sock.sendall(message)
        sock.sendall('1')
        sock.close()
    #else:
    #    sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #    #server_address = ('192.168.88.202', 1234)
    #    server_address = ('192.168.88.222', 1234)
    #    print >>sys.stderr, 'connecting to %s port %s' % server_address
    #    sock.connect(server_address)
    #    message = '0'
    #    print >>sys.stderr, 'sending "%s"' % message
        #sock.sendall(message)
        #sock.sendall('0')
    #    sock.close()
        

def main():
    rospy.init_node('mock_client', anonymous=True)
    path_sub = rospy.Subscriber('joy', Joy, joyCallback)
    #rospy.loginfo('joystick vehicle controller starting')
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
    rospy.loginfo('joystick vehicle controller finished')

if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())

