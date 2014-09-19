from bluetooth import *

server_sock=BluetoothSocket( RFCOMM )
server_sock.bind(("",3))
server_sock.listen(2)
#client_sock, client_info = server_sock.accept()
port = server_sock.getsockname()[1]

uuid = "00001101-0000-1000-8000-00805f9b34fb"

advertise_service( server_sock, "RoboSKEL-0",
                   service_id = uuid,
                   service_classes = [ uuid, SERIAL_PORT_CLASS ],
                   profiles = [ SERIAL_PORT_PROFILE ],
#                   protocols = [ OBEX_UUID ]
                    )
                   
print("Waiting for connection on RFCOMM channel {}".format(port))
try:
	client_sock, client_info = server_sock.accept()
except error:
	print("Gamhmenh pipa")
print("Accepted connection from {}".format(client_info))
data = client_sock.recv(1024)
print(data)
client_sock.sendall("1")
print("disconnected")

client_sock.close()
server_sock.close()
print ("all done")
