#! /usr/bin/env python3
from time import *
import time
import rospy
import numpy as np
import math
import serial
from std_msgs.msg import Float64MultiArray

ad1 = serial.Serial('/dev/ttyUSB4',115200)
sleep(1)

rospy.init_node('IMU_data', anonymous=True)
hand_imu_pub = rospy.Publisher('/hand_imu', Float64MultiArray, queue_size=1)

rate = rospy.Rate(1000)



while not rospy.is_shutdown():
    try:

        dataPacket1 = ad1.readline()

        dataPacket1 = str(dataPacket1,'utf-8')

        splitPacket1 = dataPacket1.split(",")

    
        hand_imu_msg = Float64MultiArray()
        hand_imu_msg.data = np.array([float(splitPacket1[0]), float(splitPacket1[1]), float(splitPacket1[2]), float(splitPacket1[3]),\
                                             float(splitPacket1[4]), float(splitPacket1[5]), float(splitPacket1[6])])
        hand_imu_pub.publish(hand_imu_msg)

        rate.sleep()

    except:
        pass