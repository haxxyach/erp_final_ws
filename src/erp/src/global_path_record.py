#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from pyproj import Proj
from math import sqrt, pow
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import NavSatFix
from morai_msgs.msg import GPSMessage
class MapMaker:

    def __init__(self):
        rospy.init_node('map_maker', anonymous=True)

        # GPS 구독자 설정
        # self.gps_sub = rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.gps_callback)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        # 경로 메시지 초기화
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        # 이전 좌표 저장용
        self.prev_x = 0
        self.prev_y = 0
        self.is_first_point = True

        # UTM 변환 설정 (52 zone 예시)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        # 오프셋 초기화 (첫 번째 GPS 좌표에서 설정)
        self.east_offset = None
        self.north_offset = None

        # 파일 저장 경로 설정
        self.file_path = "/home/unita/erp_final_ws/src/erp/src/test.txt"
        self.f = open(self.file_path, 'w')

        rate = rospy.Rate(30)  # 30Hz로 실행
        while not rospy.is_shutdown():
            rate.sleep()

        self.f.close()

    def gps_callback(self, gps_msg):
        # GPS 메시지로부터 현재 위치 가져오기 (위도, 경도를 UTM으로 변환)
        latitude = gps_msg.latitude
        longitude = gps_msg.longitude
        altitude = gps_msg.altitude

        # 위도와 경도를 UTM 좌표로 변환
        utm_x, utm_y = self.proj_UTM(longitude, latitude)

        # 첫 번째 GPS 데이터를 기준으로 오프셋 설정
        if self.is_first_point:
            self.east_offset = utm_x
            self.north_offset = utm_y
            self.prev_x = 0  # 첫 번째 좌표는 (0, 0) 기준으로
            self.prev_y = 0
            self.is_first_point = False
            rospy.loginfo(f"Offset set: east_offset={self.east_offset}, north_offset={self.north_offset}")

        # 오프셋 적용 (현재 좌표 - 오프셋)
        x = utm_x - self.east_offset
        y = utm_y - self.north_offset
        z = altitude  # 고도는 그대로 사용

        # 두 좌표 간의 거리 계산 (1cm 이상 이동했을 경우 기록)
        distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))

        if distance > 0.01:  # 1cm 이상 이동한 경우
            # 파일에 기록
            data = '{0}\t{1}\t{2}\n'.format(x, y, z)
            self.f.write(data)

            # 새로운 포인트를 경로에 추가
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0  # 기본 회전 값

            self.path_msg.poses.append(pose)  # 경로에 새로운 포즈 추가

            # 이전 좌표 업데이트
            self.prev_x = x
            self.prev_y = y

            # 좌표 및 오프셋 출력
            rospy.loginfo(f"Current Position: x={x}, y={y}, Offset: east_offset={self.east_offset}, north_offset={self.north_offset}")

if __name__ == '__main__':
    try:
        map_maker = MapMaker()
    except rospy.ROSInterruptException:
        pass
