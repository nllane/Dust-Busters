#!/usr/bin/pyton3
'''
This is the code for Basic wall following it will only look for obsticals and at a 90 degree angle left of the robot.
The code also has an attempt to signal the Arduino when the python code stops.
This code has no ablity correcting based on the angle to the wall.
'''

##library
import math
import numpy as np
import time
from time import sleep
import smbus
import RPi.GPIO as GPIO
print('I start on boot')
# Lidar
from math import cos, sin, pi, floor
import os
##from adafruit_rplidar import RPLidar
from rplidar import RPLidar
## spare wire to tell the Arduino that the code stopped
## pi sends 3.3 volts and arduino is a 5 volt system so this was not reliable but did work for a short time
GPIO.setmode(GPIO.BCM)
GPIO.setup(40,GPIO.OUT,initial=GPIO.HIGH)

## i2c communication setup
address = 0x09
bus = smbus.SMBus(1)
time.sleep(0.1)


# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)
##print('test') ## Used to debug where errors occurred
lidar.clean_input()
## This makes sure there are no unexpected values already in the lidar

# set global variables
max_distance = 0
count=0

## funtion to send the aquired vaules
def write_number(send):
    ## send the values to the arduino using i2c pins
    print(send)## View what is sent for debugging
    bus.write_i2c_block_data(address, distance_corrector, send)

## function to calculate data
def process_data(data):
    ## global variables
    global max_distance
    global count
    global distance_corrector
## setup local variables and resets them
    right=0
    left=0
    position=270
    wall = 10000## set to large value because it looks for the closest point
    cm=0
    send=[0,0,0,0]
 ## reviews all angle measurements acquired
    for angle in range(360):
        distance = data[angle]
## ignore unwanted points to reduce processing power
        if angle > 30 and angle < 270 :## 120 degrees viewed
            continue
## checks the Right side for obstacles within 30 degree angles at up to 80 cm
        if (angle > 330 and distance < 800 and distance > 0):
            left+=1
## checks the left side for obstacles within 30 degree angles at up to 80 cm
        if (angle < 30 and distance < 800 and distance > 0):
            right+=1
## checks the left side of the robot for the distance perpendicular to the robot
        if angle == 270:
            if distance!=0:
## sets distance to send to this value
                wall=distance
                position=angle
    count +=1
## adjusts the value to be in cms
    cm=int((wall)/10)
    print("wall ",cm)## Used to debug any problems with distance
## Creates a value to adjust for bit size
    distance_corrector= int(cm/255)
    print("angle ",position)
##Sends the values every 5 scans to reduce delays to the LiDAR
    if count==5:
## print statements to see size of obstacles relative to pixel size
        print('sending:')
        print(left)
        print(right)
## remove chance of seeing nonexistent obstacles by filtering out points that were not overridden
        if left >= 3:# 3 was found effective at removing errors without missing obstacles
            print("Left")# print that it is avoiding an obstacle
            send[0]=1
## remove chance of seeing nonexistent obstacles by filtering out points that were not overridden
        if right >= 3:
            print("Right")# print that it is avoiding an obstacle
            send[1]=1
        print("------------------")
## Adds the other values to the send matrix
        send[2]=cm
        send[3]=position-255 ##adjusted to allow for one byte send limit
        write_number(send)
        count=0
        
 
scan_data = [0]*360

## LiDAR code found online
try:
    for scan in lidar.iter_scans():
## runs through the scans as they are created
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)
## runs the processing code

except: ## stop command from computer or if it hits an error
 ## For debugging I would add KeyBoardInterrupt to the above code to allow for the errors to be raised and displayed
    print('Stoping.')
 ##signal that an error ocurred by setting a pin high
    GPIO.output(40,GPIO.HIGH)
 ## code to properly stop the lidar
    lidar.stop()
    lidar.stop_motor()
    lidar.clean_input()
## This makes sure there are no unexpected values already in the lidar
    lidar.disconnect()
    time.sleep(2)
## gives time for the arduino to see the flag
    GPIO.output(40,GPIO.LOW)
