    # inprots all pi camera librarys
import math
import numpy as np
##import cv2 as cv
import time
from time import sleep
from picamera import PiCamera
import smbus

    # Lidar
from math import cos, sin, pi, floor
import os
import pygame
##from adafruit_rplidar import RPLidar
from rplidar import RPLidar

address = 0x09
left=0
right=0
count=0

#set up of the camara's conection to the pi
##
##camera = PiCamera(resolution=(640, 360), framerate=30)
##camera.iso = 100
##sleep(2)
##camera.shutter_speed = camera.exposure_speed
##camera.exposure_mode = 'off'
##g = camera.awb_gains
##camera.awb_mode = 'off'
##camera.awb_gains = g
##camera.capture_sequence(['image%02d.jpg' % i for i in range(10)])

os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()
lcd = pygame.display.set_mode((320,240))
pygame.mouse.set_visible(False)
lcd.fill((0,0,0))
pygame.display.update()

bus = smbus.SMBus(1)
time.sleep(0.1)


# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar('/dev/ttyUSB0')
lidar.stop_motor()
lidar.stop()
# used to scale data to fit on the screen
max_distance = 0


def writeNumber(left,right):
    ##print('hi')
    bus.write_byte_data(address,left,right)

#pylint: disable=redefined-outer-name,global-statement
def process_data(data):
    global max_distance
    global count
    global right
    global left
    lcd.fill((0,0,0))
    pygame.display.update()
    right=0
    left=0
    sendr=0
    sendl=0
    #lcd.fill((0,0,0))
    for angle in range(360):
        distance = data[angle]
        if angle > 50 and angle < 310:
            continue
        if (angle > 330 and distance < 1200 and distance > 0):
            right+=1
        if (angle < 30 and distance < 1200 and distance > 0):
            left+=1
        if distance > 0 and distance < 600:                  # ignore initially ungathered data points
            max_distance = max([min([5000, distance]), max_distance])
            radians = angle * pi / 180.0
            x = distance * cos(radians)
            y = distance * sin(radians)
            point = (160 + int(x / max_distance * 119), 120 + int(y / max_distance * 119))
            lcd.set_at(point, pygame.Color(255, 255, 255))
    count +=1

    if count==5:
        print('sending:')
        print(left)
        print(right)
        if left >= 3:
            print("Left")
            sendl=1
        if right >= 3:
            print("Right")
            sendr=1
        print("------------------")
        writeNumber(sendl,sendr)
        count=0
        left=0
        right=0
        
        
        
                
    pygame.display.update()


def pic():
    location = '/home/nllane/screen.jpg'
    #Camera initialization and capturing an image
    camera = PiCamera(resolution=(640,360), framerate=30)
    camera.capture(location)
    img = cv.imread(location)
    camera.close()
    cv.imshow('image', img)
    cv.waitKey(0)
    cv.destroyAllWindows()
    return img
        
#sets up a funtion that can read the arduino. used for debugging
scan_data = [0]*360

try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)
except KeyboardInterrupt:
    print('Stoping.')
lidar.stop()
lidar.disconnect()
