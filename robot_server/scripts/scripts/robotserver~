#!/usr/bin/python           # This is server.py file

import socket               # Import socket module
import sys

from messageHandler import handler

try:
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)         # Create a socket object
except socket.error as msg:
	s = None
	exit(1)
#port that module listens
ip="192.168.2.104"
port = 12345 
try:		# Reserve a port for your service.
	s.bind((ip, port))# Bind to the port
except socket.error:
	print("Could not bind to ip {} port {}".fromat(ip,port))
	exit(1)

s.listen(1)                 # Now wait for client connection.
print("listening..")
while True:
	c, addr = s.accept()     # Establish connection with client.
	#id=((c.recv(1024)))#get device's name
	
	#print("Got connection from {}".format(id.decode(encoding='utf_8', errors='strict')))
	#mess="Connection established"
	#print(mess)
	#n=c.sendall(mess.encode(encoding='utf_8', errors='strict'))
	
	#call handler to handle clients requests
	x=handler(c)
	x.handle()
	#while n != None:
		#n=s.send(mess.encode(encoding='utf_8', errors='strict'))
s.close()# Close the connection
