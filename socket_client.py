#!/usr/bin/env python
import socket

i=0
HOST = 'localhost'  
PORT = 9976
ok = 0

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

None
while True:
    data = client_socket.recv(1024)
    print('Received', repr(data.decode()))

    if data.decode() == "give me position" :
        nextpose=[0.62, -0.2, ok]
        senddata=nextpose
        print(senddata)
        client_socket.sendall(senddata.encode())
        print('send : ', senddata)


    

    #client_socket.close()