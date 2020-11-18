#!/usr/bin/env python3
import math
import serial
import sys
import struct
import time
import rospy
from sensor_msgs.msg import LaserScan
rospy.init_node("lidar")
pub = rospy.Publisher("scan",LaserScan,queue_size=10)
scan = LaserScan()
preStartAngle = 0
rpm = 0
dataObtained = False
ser = serial.Serial(port = '/dev/ttyUSB0', baudrate = 115200)
HEAD1 = 0x55
HEAD2 = 0xaa
HEAD3 = 0x03
HEAD4 = 0x08
ST_HEAD1 = 0
ST_HEAD2 = 1
ST_HEAD3 = 2
ST_HEAD4 = 3
ST_SPEED = 4
writePos = 0
dataSize = 400
isInvert = True
arpm = 0
theta = [0] * dataSize
distance = [0] * dataSize
intensity = [0] * dataSize
offset = 102
Index_m = dataSize/360.0
while True:
    state = ST_HEAD1
    while True:
        if state == ST_HEAD1:
            head = ser.read()
            if int.from_bytes(head, "little") == HEAD1:
                state = ST_HEAD2
        elif state == ST_HEAD2:
            head = ser.read()
            if int.from_bytes(head, "little") == HEAD2:
                state = ST_HEAD3
            else:
                state = ST_HEAD1
        elif state == ST_HEAD3:
            head = ser.read()
            if int.from_bytes(head, "little") == HEAD3:
                state = ST_HEAD4
            else:
                state = ST_HEAD1
        elif state == ST_HEAD4:
            head = ser.read()
            if int.from_bytes(head, "little") == HEAD4:
                break
            else:
                state = ST_HEAD1
            
    tmp = ser.read(4)
    (rotationSpeedTmp, startAngleTmp) = struct.unpack_from("<2H", tmp)

    rpm = rotationSpeedTmp / 64
    if arpm == 0:
        arpm = rpm
    arpm = 0.01*rpm + 0.99*arpm
    startAngle = (startAngleTmp - 0xa000) / 64

    distanceTmp = [0] * 8
    intensityTmp = [0] * 8
    
    for i in range(8):
        tmp = ser.read(3)
        (distanceTmp[i], intensityTmp[i]) = struct.unpack_from("<HB", tmp)
    tmp = ser.read(4)
    (endAngleTmp, crc) = struct.unpack_from("<2H", tmp)
    endAngle = (endAngleTmp - 0xa000) / 64

    preStartAngle = startAngle
    startAngleRad = startAngle * math.pi / 180 * (-1 if isInvert else 1)
    endAngleRad = endAngle * math.pi / 180 * (-1 if isInvert else 1)
    angleIncrement = (endAngleRad - startAngleRad) / len(distanceTmp)
    
    if(endAngle>startAngle):
        step =(endAngle - startAngle)/8
    else:
        step =(endAngle - (startAngle - 360))/8

    for i in range(len(distanceTmp)):
        Ma = (startAngle + step*i)+(offset + 180)
        index = Ma*Index_m
        index = round(index)
        index = index % 400
        index = 399 - index
        distance[index] = distanceTmp[i]/1000.0
        intensity[index] = intensityTmp[i]

        
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = 0.0
    scan.angle_max = 2.0*3.14
    scan.angle_increment = (2*3.14)/dataSize
    scan.time_increment = 1.0/arpm/dataSize/60
    scan.range_min = 0.12
    scan.range_max = 8.0
    scan.ranges = distance
    scan.intensities = intensity
    pub.publish(scan)
    #rospy.loginfo(len(distanceTmp))
