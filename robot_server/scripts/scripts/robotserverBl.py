#!/usr/bin/env python
# Requirements:
# sudo aptitude install python-bluetooth


import socket               # Import socket module
import sys
from messageHandler import handler
from bluetooth import *

contype=sys.argv[0]
try:
	if(contype==0):
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)# Create a socket object
	else:
		s=BluetoothSocket( RFCOMM )
except socket.error as msg:
	exit(1)
#Wired IP
ip="192.168.2.104"
#Wireless IP
w_ip="192.168.2.23"

#port that module listens
port = 12345 
#UUID used to verify connection
uuid = "00001101-0000-1000-8000-00805f9b34fb"

try:		# Reserve a port for your service.
	if(contype==0):
		print("Connecting with local network")
		s.bind((ip, port))# Bind to the port
	else:
		print("Connecting with bluetooth")
		s.bind(("",3))
except socket.error:
	print("Could not bind to IP {} port {} ,attempting IP {}".format(ip,port,ip2))
	try:
		s.bind((w_ip, port))
	except socket.error:
		print("Server off line")
		exit(1)

if(contype==0):
	s.listen(1)
else:
	s.listen(1)
	port = s.getsockname()[1]
	advertise_service( s, "SKEL-0 ",service_id = uuid,
                   service_classes = [ uuid, SERIAL_PORT_CLASS ],
                   profiles = [ SERIAL_PORT_PROFILE ],
#                  protocols = [ OBEX_UUID ]
                    )	# Now wait for client connection.
print("listening..")
close=True
try:
	while close:
			c, addr = s.accept()     # Establish connection with client.
			print("accept")
			#call handler to handle clients requests
			x=handler(c)
			close=x.handle()
finally:
	s.close()# Close the connection
