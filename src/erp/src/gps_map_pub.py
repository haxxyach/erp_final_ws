#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage
from visualization_msgs.msg import Marker  # Marker 메시지 임포트

class GPS_to_UTM:
    def __init__(self):
        rospy.init_node('GPS_to_UTM', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.gps_pub = rospy.Publisher("/gps_map", Float32MultiArray, queue_size=1)
        self.marker_pub = rospy.Publisher("/gps_marker", Marker, queue_size=1)  # Marker 퍼블리셔 추가

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.utm_msg = Float32MultiArray()
        self.is_gps_data = False
        self.east_offset = 302473.3674341681
        self.north_offset = 4123735.320811661

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

        # Marker 메시지 생성 및 퍼블리시
        marker = Marker()
        marker.header.frame_id = "map"  # 프레임 ID 설정
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE  # 마커 타입 설정 (구 형태)
        marker.action = Marker.ADD
        marker.pose.position.x = map_x
        marker.pose.position.y = map_y
        marker.pose.position.z = altitude  # 고도 정보를 사용하거나 0으로 설정 가능
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        # 마커의 크기 설정
        marker.scale.x = 1.0  # 마커의 크기 (단위: 미터)
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # 마커의 색상 설정 (RGBA)
        marker.color.a = 1.0  # 투명도 (0.0부터 1.0)
        marker.color.r = 1.0  # 빨간색 성분 (0.0부터 1.0)
        marker.color.g = 0.0  # 녹색 성분
        marker.color.b = 0.0  # 파란색 성분

        # 마커 퍼블리시
        self.marker_pub.publish(marker)

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
        GPS_to_UTM()
    except rospy.ROSInterruptException:
        pass
