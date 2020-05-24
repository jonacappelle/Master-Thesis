from vpython import *
import numpy as np
from time import *
import math
import serial


# scene.range = 20
scene.width = 1500
scene.height = 800

# x = sphere(radius=5, opacity=0.5)
# xArrow = arrow(length=5, axis=vector(1,0,0), color=color.red)
# yArrow = arrow(length=5, axis=vector(0,1,0), color=color.green)
# zArrow = arrow(length=5, axis=vector(0,0,1), color=color.blue)


# ad=serial.Serial('com11',115200)
ad = serial.Serial('COM11', baudrate = 115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.1)
sleep(1)

 
scene.range=5
toRad=2*np.pi/360
toDeg=1/toRad
scene.forward=vector(1,-1,1)

scene.width=600
scene.height=600
 
xarrow=arrow(lenght=4, shaftwidth=.1, color=color.red,axis=vector(1,0,0))
yarrow=arrow(lenght=4, shaftwidth=.1, color=color.green,axis=vector(0,1,0))
zarrow=arrow(lenght=4, shaftwidth=.1, color=color.blue,axis=vector(0,0,1))
 
frontArrow=arrow(length=4,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
upArrow=arrow(length=1,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0))
sideArrow=arrow(length=2,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))

# myObj = cylinder(radius=5, pos=vector(0,0,0,))
myObj=box(length=5,width=5,height=1,opacity=.8,pos=vector(0,0,0,))
# bBoard=box(length=6,width=2,height=.2,opacity=.8,pos=vector(0,0,0,))
# bn=box(length=1,width=.75,height=.1, pos=vector(-.5,.1+.05,0),color=color.blue)
# nano=box(lenght=1.75,width=.6,height=.1,pos=vector(-2,.1+.05,0),color=color.green)
# myObj=compound([bBoard,bn,nano])
while (True):
    while (ad.inWaiting()==0):
        pass
    dataPacket=ad.readline()

    # print(dataPacket)

    dataPacket=str(dataPacket,'utf-8')
    splitPacket=dataPacket.split("\t")

    # print(splitPacket[0], splitPacket[1], splitPacket[2])

    

    roll=-float(splitPacket[0])#*toRad
    pitch=float(splitPacket[1])#*toRad
    yaw=-float(splitPacket[2])#*toRad+np.pi
    batt_percent=float(splitPacket[3])
    rssi=float(splitPacket[4])

    print("Roll=",roll*toDeg,"  Pitch=",pitch*toDeg,"   Yaw=",yaw*toDeg,"   Batt_percent=",batt_percent,"   RSSI=",rssi)
    rate(50)
    k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
    y=vector(0,1,0)
    s=cross(k,y)
    v=cross(s,k)
    vrot=v*cos(roll)+cross(k,v)*sin(roll)
 
    frontArrow.axis=k
    sideArrow.axis=cross(k,vrot)
    upArrow.axis=vrot
    myObj.axis=k
    myObj.up=vrot
    sideArrow.length=4
    frontArrow.length=4
    upArrow.length=4

