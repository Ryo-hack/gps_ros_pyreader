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
        self.status = 0
        self.service = 0
        self.GPS_ql  = 0

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

    def data_adjust(self): 
        data=[]
        data=self.garmin_GPS_data
        raw_data=data.split(',')
        mode = raw_data[0]
        UTC = int(raw_data[1])
        Latitude = float(raw_data[2])*0.01
        Lat_hem = raw_data[3]
        Longitude = float(raw_data[4])*0.01
        Log_hem = raw_data[5]
        self.GPS_ql =int(raw_data[6])
        GPS_num = raw_data[7]
        self.altitude = float(raw_data[9])

        JTC = UTC+90000
        Latitude_dec = (Latitude-int(Latitude))/0.6
        self.Latitude = int(Latitude)+Latitude_dec
        Longitude_dec = (Longitude-int(Longitude))/0.6
        self.Longitude = int(Longitude)+Longitude_dec



    def publisher(self):
        self.GPS_data.header.frame_id = 'gps'
        self.GPS_data.header.stamp = rospy.Time.now()
        
        if self.GPS_ql==0 :
            self.status = -1
        elif self.GPS_ql== 1 :
            self.status =  0
        elif self.GPS_ql== 2 :
            self.status = 2
        else :
            self.status = 1

        self.GPS_data.status.status = self.status
        self.GPS_data.status.service = self.service
        self.GPS_data.latitude  = self.Latitude
        self.GPS_data.longitude = self.Longitude
        self.GPS_data.altitude  = self.altitude
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
        if len(GPS.garmin_GPS_data) >= 66 :
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
