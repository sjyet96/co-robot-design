#!/usr/bin/env python
import urx
import socket
import math
import time
import rospy
from std_msgs.msg import Int32

### functions ###
class UR_Ctrl(object):
    """docstring for UR_Ctrl"""
    acceleration=0.5
    velocity = 0.5
    i=0
    nextpose=[0,0,0,0,0,0]
    flag=0
    camcenterpose =[0,0,0,0,0,0]
    finalboxcenterpose=[0,0,0,0,0,0]
    firstboxlinepose=[0,0,0,0,0,0]
    secondboxlinepose=[0,0,0,0,0,0]
    temppose=[0,0,0,0,0,0]
    degree=0
    rotate=0
    def __init__(self,gripper):
        self.robot = urx.URRobot("192.168.12.103", use_rt=False)
        self.HOST = '192.168.12.102'
        self.PORT = 7777
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
        self.nextpose = [0, -0.5, 0.5, 0, 3.14, 0]
        self.robot.movel(self.nextpose, acc = self.acceleration, vel=self.velocity, wait = True)
        self.getinfo() 

    def sensorcheck(self):
        print("sensor check")
        
        self.robot.set_digital_out(0, True)
        self.sensor = self.robot.get_digital_in(7)
        while self.sensor == False:
            self.sensor = self.robot.get_digital_in(7)

            if self.sensor == True:
                print("get box")
                break

        return self.sensor

    def moveconv(self):
        self.robot.set_digital_out(1, True)     ####### input 1 I/O    1->0 moving  
        self.robot.set_digital_out(1, False)


    def movetobottle(self) :
        print("movetobottle")
        self.nextpose = [-0.372, -0.417, 0.5, 0, 3.14, 0]
        self.robot.movel(self.nextpose, acc = self.acceleration, vel=self.velocity, wait = True)
        self.getinfo()

    def pickupbottle(self) :
        print("pick up bottle")
        self.pose, self.joint = self.getinfo()
        self.ctrl_gripper.gripper_ctrl(self.ctrl_gripper.GRIPPER_OPEN,0.5)
        self.robot.down(z=0.150, acc= 0.1, vel=0.1)
        self.getinfo()
        self.ctrl_gripper.gripper_ctrl(self.ctrl_gripper.GRIPPER_CLOSE,0.5)        
        self.robot.up(z=0.150,acc= 0.2, vel=0.2)    
        self.getinfo()

    def movetobox(self) :
        print("movetobox")
        self.viapose = [0, -0.5, 0.5, 0, 3.14, 0]
        self.nextpose = [0.63, -0.10, 0.5, 0, 3.14, 0]
        self.robot.movels([self.viapose,self.nextpose], acc = 0.3, vel=0.3, radius = 0.1, wait=True)
        self.getinfo()

    def putdownbottle(self) :
        print("putdown")
        self.robot.down(z=0.285,acc= 0.1, vel=0.1)
        self.getinfo()
        self.ctrl_gripper.gripper_ctrl(self.ctrl_gripper.GRIPPER_OPEN,0.5)
        self.robot.up(z=0.285,acc= 0.2, vel=0.2)
        self.getinfo()
        self.ctrl_gripper.gripper_ctrl(self.ctrl_gripper.GRIPPER_CLOSE,0.5)

    def rotatewirst(self, degree):
        print("rotate wirst", degree)
        self.degree=float(degree)
        self.current_pose , self.current_joint = self.getinfo()
        self.next_joint=self.current_joint
        self.next_joint[5]-=math.radians(self.degree)
        self.robot.movej(self.next_joint , acc = 0.5, vel=0.6, wait = True)
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
                print(self.temppose) ####[x,y,ry,ok]###
                self.nextpose=[0,0,0.5,0,3.14,0]
                self.nextpose[0]=float(self.temppose[0])/1000
                self.nextpose[1]=float(self.temppose[1])/1000
                print(self.nextpose)
                self.robot.movel(self.nextpose, acc = 0.05, vel=0.05, wait = True)
                print("move next")
                if self.temppose[3] == 'ok' :
                    self.degree = self.temppose[2]
                    self.camcenterpose=self.nextpose
                    self.finalboxcenterpose = [self.camcenterpose[0]+0.03,self.camcenterpose[1]-0.0608,self.camcenterpose[2],self.camcenterpose[3],self.camcenterpose[4],self.camcenterpose[5]]
                    self.robot.movel(self.finalboxcenterpose, acc = 0.1, vel=0.1, wait = True)

                    break


        # while True :
        #     self.data = "finish"
        #     self.client_socket.sendall(self.data.encode()) 
        #     self.data = self.client_socket.recv(1024)
        #     if self.data :
        #         self.rescieve_data =self.data.decode()


        #         if self.rescieve_data == 'do':
        #             print ("do")
        #             self.degree_box = float(self.degree)
        #             break;
        #         else :
        #             print("rotate 90")
        #             self.degree_box = 90-float(self.degree)
        #             #self.rotatewirst(self.degree_box)
        #             break;


    def firstBoxLinePosition(self):

        # if self.rotate == 1 :
        #     self.degree = self.degree*3.1415/180
        #     self.rotateboxcenterpose = self.robot.getl()

        #     self.firstboxlinepose = [self.rotateboxcenterpose[0]+0.0225*math.sin(self.degree),self.rotateboxcenterpose[1]-0.0225*math.cos(self.degree),0.5,self.rotateboxcenterpose[3],self.rotateboxcenterpose[4],self.rotateboxcenterpose[5]]
        #     print('after : ',self.firstboxlinepose)
        #     self.robot.movel(self.firstboxlinepose, acc = 0.05, vel=0.05, wait = True)
        #     self.getinfo()

        # else :
        self.firstboxlinepose = [self.finalboxcenterpose[0],self.finalboxcenterpose[1]+0.0205,0.5,0,3.14,0]
        self.robot.movel(self.firstboxlinepose, acc = 0.1, vel=0.1, wait = True)


    def secondBoxLinePosition(self) :
        
        # if self.rotate == 1 :
        #     self.robot.movel(self.rotateboxcenterpose, acc = 0.05, vel=0.05, wait = True)
        #     self.degree = self.degree*3.1415/180
        #     self.secondboxlinepose = [self.rotateboxcenterpose[0]-0.0225*math.sin(self.degree),self.rotateboxcenterpose[1]+0.0225*math.cos(self.degree),0.5,self.rotateboxcenterpose[3],self.rotateboxcenterpose[4],self.rotateboxcenterpose[5]]
        #     print('after : ',self.firstboxlinepose)
        #     self.robot.movel(self.secondboxlinepose, acc = 0.05, vel=0.05, wait = True)
        #     self.getinfo()

        # else :
        self.robot.movel(self.finalboxcenterpose, acc = 0.1, vel=0.1, wait = True)
        self.secondboxlinepose = [self.finalboxcenterpose[0],self.finalboxcenterpose[1]-0.022,0.5,0,3.14,0]
        self.robot.movel(self.secondboxlinepose, acc = 0.1, vel=0.1, wait = True)

    # def findAngle(self):
    #     print("find angle")
    #     self.rotatewirst(self.degree_box)
    #     self.rotate = 1

        

    def start_ctrl(self):
        self.client_socket, self.addr = self.server_socket.accept()
        print('Connected by', self.addr)
    
    def end_ctrl(self):
        self.client_socket.close()
        self.server_socket.close()
        self.robot.close()