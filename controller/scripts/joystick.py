#!/usr/bin/env python3
from time import sleep
import time
import numpy as np
import rospy
import RPi.GPIO as GPIO
from sensor_msgs.msg import JointState
from multiprocessing import Process, Array, Value, Queue
import multiprocessing


ValveFront = 17
ValveRear = 27
enPinR = 13
directionpinR = 19
steppinR = 26
enPinL  = 16
directionpinL = 20
steppinL = 21
FORWARD = GPIO.HIGH
BACKWARD = GPIO.LOW
CLOCKWISE = GPIO.HIGH
COUNTERCLOCK = GPIO.LOW
stepsPerRevolution = 6400
OUTPUT = GPIO.OUT
# pin = 12

lspeed = 100
rotateSpeed = 50

rotationCountReader = Value("f",0)
rotationCountReader_ = Value("f",0)
linearCountReader = Value("f",0)
linearCountReader_ = Value("f",0)
# msg_ = np.zeros((7,),np.float16)
msg_ = Array("f",[0,0,0,0,0,0,0,0,0,0,0,0,0,0])

qu = Queue()
lock = multiprocessing.Lock()

def delayMicroseconds(sec):
    start = time.perf_counter()
    tmp = sec/1000000.0
    while (time.perf_counter()-start) < tmp:
        pass

def setupBoard():
    GPIO.setmode(GPIO.BCM)
    # GPIO.setup(pin,GPIO.OUT)
    GPIO.setwarnings(False)
    GPIO.setup (ValveFront,OUTPUT)
    GPIO.setup(ValveRear,OUTPUT)
    GPIO.setup (directionpinR,OUTPUT)
    GPIO.setup (steppinR,OUTPUT)
    GPIO.setup(enPinR,OUTPUT)
    GPIO.setup (directionpinL,OUTPUT)
    GPIO.setup (steppinL,OUTPUT)
    GPIO.setup(enPinL,OUTPUT)
    GPIO.output(ValveFront,GPIO.LOW)
    GPIO.output(ValveRear,GPIO.LOW)
    GPIO.output(enPinR,GPIO.LOW)
    GPIO.output(enPinL,GPIO.LOW)

def Lmotor(directionL:bool,dis:float):
    speed_m = 400.0/lspeed
    GPIO.output(directionpinL,directionL)
    for i in range(int(stepsPerRevolution*dis/6)): # stepsPerRevolution*dis/6
        GPIO.output (steppinL,GPIO.HIGH)
        delayMicroseconds(50)
        # sleep(0.005)
        GPIO.output (steppinL,GPIO.LOW)
        delayMicroseconds(50)
        if(directionL == FORWARD):
            linearCountReader.value = linearCountReader.value+1
        else:
            linearCountReader.value = linearCountReader.value-1
        # sleep(0.005)

def LmotorD(directionL:bool):
    # se.acquire()
    GPIO.output(directionpinL,directionL)
    for i in range(2): # stepsPerRevolution*dis/6
        GPIO.output (steppinL,GPIO.HIGH)
        delayMicroseconds(50)
        # sleep(0.005)
        GPIO.output (steppinL,GPIO.LOW)
        delayMicroseconds(50)
        if(directionL == FORWARD):
            linearCountReader.value = linearCountReader.value+1
        else:
            linearCountReader.value = linearCountReader.value-1
    # qu.put(linearCountReader.value)
    # se.release()

def Rmotor(directionR = CLOCKWISE):
    GPIO.output(directionpinR,directionR)
    for i in range(int(stepsPerRevolution/4)):
        GPIO.output (steppinR,GPIO.HIGH)
        delayMicroseconds(rotateSpeed)
        # sleep(0.005)
        GPIO.output (steppinR,GPIO.LOW)
        delayMicroseconds(rotateSpeed)
        if(directionR == CLOCKWISE):
            rotationCountReader.value = rotationCountReader.value+1
        else:
            rotationCountReader.value = rotationCountReader.value-1
        # sleep(0.005)

def RmotorD(directionR = CLOCKWISE):
    GPIO.output(directionpinR,directionR)
    for i in range(2):
        GPIO.output (steppinR,GPIO.HIGH)
        delayMicroseconds(rotateSpeed)
        # sleep(0.005)
        GPIO.output (steppinR,GPIO.LOW)
        delayMicroseconds(rotateSpeed)
        if(directionR == CLOCKWISE):
            rotationCountReader.value = rotationCountReader.value+1
        else:
            rotationCountReader.value = rotationCountReader.value-1

def RotationThread():
    # global msg_
    while True:
        # print(stepsPerRevolution)
        if msg_[0] >= 1:
            Rmotor(CLOCKWISE)
            sleep(0.001)
        if msg_[0] == -1:
            Rmotor(COUNTERCLOCK)
            sleep(0.001)
        if msg_[5] > 50:
            RmotorD(CLOCKWISE)
        if msg_[5] < -50:
            RmotorD(COUNTERCLOCK)

def LinearThread():
    while True:
        if msg_[4] >= 50:
            GPIO.output(ValveFront,GPIO.LOW)
            GPIO.output(ValveRear,GPIO.HIGH)
            sleep(0.02)
            Lmotor(BACKWARD,10)
            GPIO.output(ValveFront,GPIO.HIGH)
            GPIO.output(ValveRear,GPIO.LOW)
            sleep(0.02)
            Lmotor(FORWARD,10)
            sleep(0.001)
        if msg_[4] <= -50:
            GPIO.output(ValveFront,GPIO.LOW)
            GPIO.output(ValveRear,GPIO.HIGH)
            sleep(0.02)
            Lmotor(FORWARD,10)
            GPIO.output(ValveFront,GPIO.HIGH)
            GPIO.output(ValveRear,GPIO.LOW)
            sleep(0.02)
            Lmotor(BACKWARD,10)
            sleep(0.001)

        if msg_[6] > 50:
            LmotorD(BACKWARD)
        if msg_[6] < -50:
            LmotorD(FORWARD)
        

def doMsg(msg:JointState):
    for i in range(14):
        msg_[i] = msg.position[i]


def ft_sub():
    rospy.init_node('mediator',anonymous=True)
    sub = rospy.Subscriber('joystick',JointState,doMsg,queue_size=10)
    rate = rospy.Rate(500)
    
    while not rospy.is_shutdown():
        # se.acquire()
        # print("linear count number is:",value)
        # print("rotation count number is:",rotationCountReader.value)
        
        # se.release()
        rate.sleep()

if __name__=='__main__':
    try:
        setupBoard()
        a1 = Process(target=RotationThread)
        a2 = Process(target=LinearThread)
        a3 = Process(target=ft_sub)
        a1.start()
        a2.start()
        a3.start()
        a1.join()
        a2.join()
        a3.join()
        print("----------------")
    except KeyboardInterrupt:
        GPIO.cleanup()
        
