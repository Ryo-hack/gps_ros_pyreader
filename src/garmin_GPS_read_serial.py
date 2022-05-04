#!/usr/bin/env python
# -*- coding: utf-8 -*-
from re import L
import rospy
import sys
import rosparam
import time
import serial
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import  NavSatStatus

class GPS :
    def __init__(self):
        self.garmin_GPS_data  = 0.0
        self.pub = rospy.Publisher("GPS", NavSatFix, queue_size = 10)
        self.GPS_data = NavSatFix()
        self.ser = 0
        self.Latitude = 0.0
        self.Longitude = 0.0
        self.altitude = 0.0

    #def Subscribers(self):
    #    self.sub = rospy.Subscriber('force_sensor_data', Vector3, self.callback)

    def set_serial(self):
        port_name = rospy.get_param('~port','/dev/ttyUSB0')
        if len(sys.argv) == 2 :
            port_name  = sys.argv[1]
        self.ser = serial.Serial(port_name,9600,timeout=0.1)
        #rospy.loginfo("Connected on %s" % (port_name) )
        time.sleep(1)
        return self.ser

    def serial_read(self):
        #self.ser = serial.Serial('/dev/ttyUSB0',9600,timeout=0.1)
        self.garmin_GPS_data = self.ser.readline()
        if len(self.garmin_GPS_data) == 74 :
            self.data_adjust()
            rospy.loginfo(self.garmin_GPS_data)
            self.publisher()
        elif len(self.garmin_GPS_data) == 0 :
            rospy.loginfo(self.garmin_GPS_data)
            rospy.loginfo("GPS LOST")
        else :
            rospy.loginfo(self.garmin_GPS_data)
            rospy.loginfo("Data corruption")

        self.ser.close()
        return self.GPS_data

    #def callback(self):

    def data_adjust(self):
        data=[]
        data=self.garmin_GPS_data
        self.Latitude = (float(data[16:25]))*0.01
        self.Longitude = (float(data[28:38]))*0.01  
        self.altitude = 0.0
        rospy.loginfo(data)


    def publisher(self):
        self.serial_read()
        Fix = NavSatStatus()
        #Fix.status = GPS.mode
        #Fix.service = GPS.numSat
        self.GPS_data.header.stamp = rospy.Time.now()
        self.GPS_data.status = Fix
        self.GPS_data.latitude  = self.Latitude
        self.GPS_data.longitude = self.Longitude
        self.GPS_data.altitude  = 0.0
        self.GPS_data.position_covariance = (0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.GPS_data.position_covariance_type = 0
        rospy.loginfo(self.GPS_data)
        self.pub.publish(self.GPS_data)

if __name__ == '__main__':
    rospy.init_node("garmin_GPS_talker")
    GPS = GPS()
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        GPS.set_serial()
        GPS.serial_read()
        
        rate.sleep()
