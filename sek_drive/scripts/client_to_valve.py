#!/usr/bin/env python
import socket
import sys
import roslib
import time
import rospy
import actionlib
import signal
#import error

try:
    from sensor_msgs.msg import Joy
except ImportError:
    from joy.msg import Joy

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C - or killed me with -2'
    sys.exit(0)
    
signal.signal(signal.SIGINT, signal_handler)

def main():
        rospy.init_node('client_to_valve', anonymous=True)
        print'Started'
        myHost = rospy.get_param('/valve_ip', '192.168.88.230')
        myPort = rospy.get_param('/valve_port', 8081)
        try:
            while True :
                while True:
                    try:
                        x = rospy.get_param('/test_aligned')
                        time.sleep(.1)
                        print 'Checking If Aligned'
                    #except KeyboardInterrupt:
                    #    print "Ctrl-c pressed, Exiting"
                    #    sys.exit(1)
                    except:
                        print 'could not read ROS parameters'
                        time.sleep(.1)
                if (x==1):
                    print 'Aligned, Sending Signal To Valve'
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    server_address=(myHost, myPort)
                    print'connecting to ' + str(myHost) + ':' + str(myPort)
                    while True:
                        try:
                            sock.connect(server_address)
                            print 'connected to ' + str(myHost) + ':' + str(myPort) + '\n'
                            message = '0'
                            print 'sending ' + message
                            break
                        #except KeyboardInterrupt:
                        #    print "Ctrl-c pressed, Exiting"
                        #    sys.exit()
                        except socket.error, e:
                            print "Address-related error connecting to server: %s" % e
                            time.sleep(.1)
                    while True:
                        try:
                            sock.sendall(message)
                            message=''
                            print 'Sent Signal, Waiting for Release'
                            break
                        #except KeyboardInterrupt:
                        #    print "Ctrl-c pressed, Exiting"
                        #    sys.exit()
                        except:
                            print 'Could Not Send Signal To Valve'
                            time.sleep(.1)
                    while True:
                        try:
                            recv_data = sock.recv(256)
                            if recv_data == ('theend'):
                                print 'Got Release, Cup Is Full'
                                rospy.set_param('/test_aligned', 0)
                                time.sleep(10)
                                break
                        #except KeyboardInterrupt:
                        #    print "Ctrl-c pressed, Exiting"
                        #    sys.exit()
                        except:
                            print 'Cannot Get Release, Will Try Again'
                            time.sleep(.1)
        except KeyboardInterrupt:
            sys.exit()
        except:
            time.sleep(.1)

if __name__ == '__main__':
    # run main function and exit
    main()
    sys.exit()
