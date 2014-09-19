#!/usr/bin/env python
import time
import socket
import sys
import roslib
import struct
import rospy
import actionlib

def main():
    rospy.init_node('prox_flex_sek', anonymous=True)
    #try:
    #    rospy.spin()
    #except rospy.ROSInterruptException: pass
    #rospy.loginfo('joystick vehicle controller finished')
    print'Started'
	#rospy.loginfo('joystick vehicle controller starting')
    
    print 'Setting Host'
    myHost = '192.168.88.202'
    myPort = 8085

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # create a TCP socket
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind((myHost, myPort))
    except socket.error, msg:
        print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]# bind it to the server port
        sys.exit()
    
    
    s.listen(5)# allow 5 simultaneous
                # pending connections
    print 'Listening'
	
    # wait for nremove %d from string c++ext client to connectconnection, address = s.accept() # connection is a new socket
    while True:
        print 'in loop'
        conn, addr=s.accept()
        print 'Connected with ' + addr[0] + ':' + str(addr[1])
        while True:
            try:
                data = conn.recv(256) # receive up to 1K bytes
                print data
                if not data :
                    print "connection closed"
                    conn.close()
                else:
                    x=data.split(',')
                    print x
                    if (len(x)==2):
                        #x=data.split(',')
                        print x
                        #print data + '\n'
                        if ((x[0]=='52') and (x[1]=='52')):
                            rospy.set_param('/cupinplace',43)
                            print 'Getting ROSPARAM'
                            print rospy.get_param('/cupinplace')
                            time.sleep(4)
                        
            except:
                print("Can't receive data")
                 
            
    # close socket

if __name__ == '__main__':
    #print'oeoeoeo'
    # run main function and exit
    sys.exit(main())
