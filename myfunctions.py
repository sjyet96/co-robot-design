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
    acceleration=0.3
    velocity = 0.3
    i=0
    nextpose=[0,0,0,0,0,0]
    def __init__(self,gripper):
        self.robot = urx.URRobot("192.168.12.101", use_rt=False)
        self.HOST = 'localhost'
        self.PORT = 9976
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.HOST, self.PORT))
        self.server_socket.listen(10)
        
        self.ctrl_gripper = gripper
    
    def getinfo(self) :
        self.pose = self.robot.getl()
        self.joint = self.robot.getj()
        print("pose : ", self.robot.getl())
        print("joint : ", self.robot.getj())

        return self.pose, self.joint
        
    def start(self):
        print("start")
        self.nextpose = [0, -0.5, 0.5, 3.141, 0, 0]
        self.robot.movel(self.nextpose, acc = self.acceleration, vel=self.velocity, wait = True)
        self.getinfo() 

    def movetobottle(self) :
        print("movetobottle")
        self.nextpose = [-0.37, -0.45, 0.5, 3.14, 0, 0]
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
        self.viapose = [0, -0.5, 0.5, 3.14, 0, 0]
        self.nextpose = [0.63, -0.2, 0.5, 3.14, 0, 0]
        self.robot.movels([self.viapose,self.nextpose], acc = 0.1, vel=0.1, radius = 0.1, wait=True)
        self.getinfo()

    def putdownbottle(self) :
        print("movetobox")
        self.robot.down(z=0.295,acc= 0.1, vel=0.1)
        self.getinfo()
        self.ctrl_gripper.gripper_ctrl(self.ctrl_gripper.GRIPPER_OPEN,0.5)
        self.robot.up(z=0.295,acc= 0.1, vel=0.1)
        self.getinfo()
        self.ctrl_gripper.gripper_ctrl(self.ctrl_gripper.GRIPPER_CLOSE,0.5)

    def rotatewirst(self,degree):
        print("rotate wirst", degree)
        self.current_pose , self.current_joint = self.getinfo()
        self.next_joint=self.current_joint
        self.next_joint[5]-=math.radians(degree)
        self.robot.movej(self.next_joint , acc = 0.5, vel=0.5, wait = True)
        self.getinfo()

    def findBoxPosition(self) :
        while True:
            self.data = "give me position"
            self.client_socket.sendall(self.data.encode()) 
            self.data = self.client_socket.recv(1024)
            print('Received from', self.addr, self.data.decode())
            self.rescieve_data =self.data.decode()
            self.temppose = list(map(float,self.rescieve_data.split()))

            if temppose == [2]:
                break
            self.nextpose=[0, 0, 0.5, 3.14, 0, 0]
            self.nextpose[0]=self.temppose[0]
            self.nextpose[1]=self.temppose[1]
            print(self.nextpose)
            self.robot.movel(self.nextpose, acc = 0.5, vel=0.1, wait = True)
            pose, joint = self.getinfo()
            self.data = pose
            self.client_socket.sendall(self.data.encode()) 
            print("move next")
    
    def start_ctrl(self):
        self.client_socket, self.addr = self.server_socket.accept()
        print('Connected by', self.addr)
    
    def end_ctrl(self):
        self.client_socket.close()
        self.server_socket.close()
        self.robot.close()