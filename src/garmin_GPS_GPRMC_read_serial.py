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

    def serial_read(self):
        port_name = rospy.get_param('~port','/dev/ttyUSB0')
        if len(sys.argv) == 2 :
            port_name  = sys.argv[1]
        self.ser = serial.Serial(port_name,9600,timeout=0.1)
        #rospy.loginfo("Connected on %s" % (port_name) )
        time.sleep(1)
        self.garmin_GPS_data = self.ser.readline()
        self.ser.close()
        return self.GPS_data

    #def callback(self):

    def data_adjust(self):
        data=[]
        data=self.garmin_GPS_data
        #self.Latitude = (float(data[16:25]))*0.01
        Latitude = int(data[16:18])
        Latitude_dec = (float(data[18:25])*0.01)/0.60
        self.Latitude = Latitude + Latitude_dec

        #self.Longitude = (float(data[28:38]))*0.01
        Longitude = int(data[28:31])
        Longitude_dec = (float(data[31:38])*0.01)/0.60
        self.Longitude = Longitude + Longitude_dec

        self.altitude = 0.0



    def publisher(self):
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
    
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        GPS.serial_read()
        if len(GPS.garmin_GPS_data) == 74 :
            GPS.data_adjust()
            rospy.loginfo(GPS.garmin_GPS_data)
            GPS.publisher()
            
        elif len(GPS.garmin_GPS_data) == 0 :
            rospy.loginfo(GPS.garmin_GPS_data)
            rospy.loginfo("GPS LOST")

        else :
            rospy.loginfo(GPS.garmin_GPS_data)
            rospy.loginfo("Data corruption")

        rate.sleep()
