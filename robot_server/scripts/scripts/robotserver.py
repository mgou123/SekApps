#!/usr/bin/env python


import socket               # Import socket module

from messageHandler import handler

try:
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)         # Create a socket object
except socket.error as msg:
	s = None
	exit(1)
#port that module listens
ip="192.168.2.104"
ip2="192.168.2.23"
port = 12345 
try:		# Reserve a port for your service.
	s.bind((ip, port))# Bind to the port
except socket.error:
	print("Could not bind to IP {} port {} ,attempting IP {}".format(ip,port,ip2))
	try:
		s.bind((ip2, port))
	except socket.error:
		print("Server off line")
		exit(1)

s.listen(1)                 # Now wait for client connection.
print("listening..")
close=True
try:
	while close:
		c, addr = s.accept()     # Establish connection with client.
		
		#call handler to handle clients requests
		x=handler(c)
		close=x.handle()
finally:
	s.close()# Close the connection

