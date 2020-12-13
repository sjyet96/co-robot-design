#!/usr/bin/env python
import urx
import socket
import math
import time
#import rospy
#from std_msgs.msg import Int32

### functions ###
class UR_Ctrl(object):
    """docstring for UR_Ctrl"""
    acceleration=0.2
    velocity = 0.2
    i=0
    nextpose=[0,0,0,0,0,0]
    def __init__(self):
        print("start")
        self.robot = urx.URRobot("192.168.10.121", use_rt=False)
        self.HOST = 'localhost'
        self.PORT = 9999
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.HOST, self.PORT))
        self.server_socket.listen(10)
        
        #self.ctrl_gripper = gripper
    
    def getinfo(self) :
        self.pose = self.robot.getl()
        self.joint = self.robot.getj()
        print("pose : ", self.robot.getl())
        print("joint : ", self.robot.getj())

        return self.pose, self.joint
        
    def start(self):
       #self.boxsensorinput = self.robot.
        print("start")
        self.nextpose = [0, -0.5, 0.5, 0, 3.14, 0]
        self.robot.movel(self.nextpose, acc = self.acceleration, vel=self.velocity, wait = True)
        self.getinfo() 

    def sensorcheck(self):
        print("sensor check")
        self.robot.set_digital_out(7, True)
        self.sensor = self.robot.get_digital_in(7)
        while self.sensor == False:
            self.sensor = self.robot.get_digital_in(7)

            if self.sensor == True:
                print("get box")
                break
            

    def movetobottle(self) :
        print("movetobottle")
        self.nextpose = [-0.37, -0.45, 0.5, 0, 3.14, 0]
        self.robot.movel(self.nextpose, acc = self.acceleration, vel=self.velocity, wait = True)
        self.getinfo()

    def pickupbottle(self) :
        print("pick up bottle")
        self.pose, self.joint = self.getinfo()
        self.ctrl_gripper.gripper_ctrl(self.ctrl_gripper.GRIPPER_OPEN,0.5)
        self.robot.down(z=0.150, acc= 0.1, vel=0.1)
        self.getinfo()
        self.ctrl_gripper.gripper_ctrl(self.ctrl_gripper.GRIPPER_CLOSE,0.5)        
        self.robot.up(z=0.150,acc= 0.1, vel=0.1)    
        self.getinfo()

    def movetobox(self) :
        print("movetobox")
        self.viapose = [0, -0.5, 0.5, 0, 3.14, 0]
        self.nextpose = [0.63, -0.2, 0.5, 0, 3.14, 0]
        self.robot.movels([self.viapose,self.nextpose], acc = 0.1, vel=0.1, radius = 0.1, wait=True)
        self.getinfo()

    def putdownbottle(self) :
        print("movetobox")
        self.robot.down(z=0.295,acc= 0.1, vel=0.1)
        self.getinfo()
      #  self.ctrl_gripper.gripper_ctrl(self.ctrl_gripper.GRIPPER_OPEN,0.5)
        self.robot.up(z=0.295,acc= 0.1, vel=0.1)
        self.getinfo()
      #  self.ctrl_gripper.gripper_ctrl(self.ctrl_gripper.GRIPPER_CLOSE,0.5)

    def rotatewirst(self,degree):
        print("rotate wirst", degree)
        self.current_pose , self.current_joint = self.getinfo()
        self.next_joint=self.current_joint
        self.next_joint[5]-=math.radians(degree)
        self.robot.movej(self.next_joint , acc = 0.5, vel=0.5, wait = True)
        self.getinfo()

    def findBoxPosition(self) :
        print("findbox")
        while True :  
            self.data = "give me position"
            self.client_socket.sendall(self.data.encode()) 
            self.data = self.client_socket.recv(1024)
            if self.data:
                print('Received from', self.addr, self.data.decode())
                self.rescieve_data =self.data.decode()

                if  self.rescieve_data == 'answer':
                    break
        while True:
            #send pose       
            self.pose, self.joint = self.getinfo()
            self.sendinfodata = str(self.pose[0])+','+str(self.pose[1])+','+str(self.pose[2])
            print("send data : ", self.data)
            while True :

                self.client_socket.sendall(self.sendinfodata.encode())
                self.recieveddata = self.client_socket.recv(1024)
                
                print('Received from', self.addr, self.recieveddata.decode())
                self.rescieve_data =self.data.decode()

                if  self.rescieve_data == 'answer':
                    break 
            print("send pose")

            #recive pose
            self.recievedinfodata = self.client_socket.recv(1024)
            self.rescieve_data =self.recievedinfodata.decode()
            print("infodata recived")
            print('Received from', self.addr, self.recievedinfodata.decode())


            if self.rescieve_data:
                self.answer = 'answer'
                self.client_socket.sendall(self.answer.encode()) 
                print("send answer")

                self.temppose = self.rescieve_data.split(',')
                self.nextpose=[0,0,0.5,0,3.14,0]
                self.nextpose[0]=float(self.temppose[0])/1000
                self.nextpose[1]=float(self.temppose[1])/1000
                print(self.nextpose)
                self.robot.movel(self.nextpose, acc = 0.05, vel=0.05, wait = True)
                print("move next")
                if self.temppose[2] == 'ok' :
                    self.finalpose = [self.nextpose[0]+0.03,self.nextpose[1]-60.8/1000,self.nextpose[2],self.nextpose[3],self.nextpose[4],self.nextpose[5]]
                    self.robot.movel(self.finalpose, acc = 0.05, vel=0.05, wait = True)
                    break


            
        
    def start_ctrl(self):
        self.client_socket, self.addr = self.server_socket.accept()
        print('Connected by', self.addr)
    
    def end_ctrl(self):
        self.client_socket.close()
        self.server_socket.close()
        self.robot.close()
