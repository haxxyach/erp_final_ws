#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage
# from sensor_msgs.msg import NavSatFix


class GPS_to_UTM:
    def __init__(self):
        rospy.init_node('GPS_to_UTM', anonymous=True)
        # self.gps_sub = rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.gps_callback)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.gps_pub = rospy.Publisher("/gps_map", Float32MultiArray, queue_size=1)  # 퍼블리셔를 /gps_map으로 변경
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.utm_msg = Float32MultiArray()
        self.is_gps_data = False
        self.east_offset = 302473.63292851474
        self.north_offset = 4123735.2124132207

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_gps_data:
                print("[1] can't subscribe '/gps' topic... \n    please check your GPS sensor connection")

            self.is_gps_data = False
            rate.sleep()

    def gps_callback(self, gps_msg):
        self.is_gps_data = True
        latitude = gps_msg.latitude
        longitude = gps_msg.longitude
        altitude = gps_msg.altitude
        utm_xy = self.proj_UTM(longitude, latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]
        map_x = utm_x - self.east_offset
        map_y = utm_y - self.north_offset

        # 좌표 데이터를 Float32MultiArray로 퍼블리시
        self.utm_msg.data = [map_x, map_y]
        self.gps_pub.publish(self.utm_msg)

        # 콘솔 출력
        os.system('clear')
        print(f''' 
        ----------------[ GPS data ]----------------
            latitude    : {latitude}
            longitude   : {longitude}
            altitude    : {altitude}

                             |
                             | apply Projection (utm 52 zone)
                             V

        ------------------[ utm ]-------------------
              utm_x     : {utm_x}
              utm_y     : {utm_y}

                             |
                             | apply offset (east and north)
                             V
              
        ------------------[ map ]-------------------
        simulator map_x : {map_x}
        simulator map_y : {map_y}
        ''')

if __name__ == '__main__':
    try:
        GPS_to_UTM = GPS_to_UTM()
    except rospy.ROSInterruptException:
        pass
