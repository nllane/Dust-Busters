    # inprots all pi camera librarys
import math
import numpy as np
##import cv2 as cv
import time
from time import sleep
from picamera import PiCamera
import smbus
import RPi.GPIO as GPIO

    # Lidar
from math import cos, sin, pi, floor
import os
import pygame
##from adafruit_rplidar import RPLidar
from rplidar import RPLidar
## spare wire to tell the Arduino that the code stopped
GPIO.setmode(GPIO.BCM)
GPIO.setup(40,GPIO.OUT,initial=GPIO.HIGH)


# i2c set location
address = 0x09
bus = smbus.SMBus(1)
time.sleep(0.1)


# Setup the RPLidar
## this is the most reacent error that is ocurring with the seril failing
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)
print('test')
lidar.clean_input()
time.sleep(1)

# used to scale data to fit on the screen
max_distance = 0
count=0

## i2c bus function
def writeNumber(send):
    ##print('hi')
    print(send)
    bus.write_i2c_block_data(address, distance_correcter, send)

#pylint: disable=redefined-outer-name,global-statement
## process data to usable outcome
def process_data(data):
    global max_distance
    global count
    global distance_correcter
   ## lcd.fill((0,0,0))
   ## pygame.display.update()
    right=0
    left=0
    position=270
    wall = 10000
    cm=0
    send=[0,0,0,0]
## reviews all angle measurments aquired
    for angle in range(360):
        distance = data[angle]
## ignore unwanted points to reduce processing power
        if angle > 30 and angle < 270 :
            continue
## checks the Right side for obsticals 30 degree angles and 80 cm
        if (angle > 330 and distance < 800 and distance > 0):
            right+=1
## checks the left side for obsticals 30 degree angles and 80 cm
        if (angle < 30 and distance < 800 and distance > 0):
            left+=1
## pi game to display room while coding
##        if distance > 0 and distance < 600:                  # ignore initially ungathered data points
##            max_distance = max([min([5000, distance]), max_distance])
##            radians = angle * pi / 180.0
##            x = distance * cos(radians)
##            y = distance * sin(radians)
##            point = (160 + int(x / max_distance * 119), 120 + int(y / max_distance * 119))
##            lcd.set_at(point, pygame.Color(255, 255, 255))

## locates the closest wall
        if angle >= 270 and angle <=330: ## field of view
            if distance <= wall and distance!=0: ## checks if new distance > old
                wall=distance
                position=angle
    count +=1
    ##math to correct to a cm distance and for larger corections
    cm=int((wall)/10)
    print("wall ",cm)
    distance_correcter= int(cm/255) ## allows larger values to accurately be sent
    print("angle ",position)
    if count==5: ## send every 5 scans
## print statments to see size of obsticals relitive to pixil size
##        print('sending:')
##        print(left)
##        print(right)
## remove chance of seeing non existant obsticals with a band to filter out points that were not overriden
        if left >= 3:# 3 was found efective at removing errors without missing obsticals
            print("Left")# print that it is avoiding an obstical
            send[0]=1
## remove chance of seeing non existant obsticals with a band to filter out points that were not overriden
        if right >= 3:
            print("Right")# print that it is avoiding an obstical
            send[1]=1
        print("------------------")
        send[2]=cm
        send[3]=position-255 ## corrects to send angle within a bit of data
        writeNumber(send) ## send the array of values
        count=0
        
        
        
## updates image on the screen            
    ##pygame.display.update()
 
#sets up a funtion that can read the arduino. used for debugging
scan_data = [0]*360


## lidar code found online
try:
    for scan in lidar.iter_scans(): ##infinet run through scans because it keeps adding more
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data) ## run above function
except: ## stop command from computer
    print('Stoping.')
##    GPIO.output(40,GPIO.LOW)
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    time.sleep(2)
 ##   GPIO.output(40,GPIO.HIGH)
