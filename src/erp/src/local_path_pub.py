#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from math import sqrt, cos, sin
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class path_pub:

    def __init__(self):
        rospy.init_node('path_pub', anonymous=True)

        # 토픽 구독자 설정
        rospy.Subscriber("/gps_map", Float32MultiArray, self.gps_callback)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)  # IMU 데이터를 구독하여 yaw 값 추출

        # 로컬 패스 생성 시 사용할 변수 초기화
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'
        self.is_status = False  # 경로 계산 가능 여부 플래그
        self.local_path_size = 50  # 로컬 경로 크기

        self.x = 0  # 현재 위치 x 좌표
        self.y = 0  # 현재 위치 y 좌표
        self.yaw = 0  # 차량의 헤딩 (yaw 값)

        rate = rospy.Rate(20)  # 20Hz
        while not rospy.is_shutdown():
            if self.is_status:
                self.process_local_path()
            rate.sleep()

    def process_local_path(self):
        x = self.x
        y = self.y
        yaw = self.yaw

        # 차량의 진행 방향 벡터 계산 (글로벌 좌표계 기준)
        heading_vector = (cos(yaw), sin(yaw))
        local_points = []

        # 글로벌 경로에서 로컬 좌표로 변환된 경로 점 선택
        for waypoint in self.global_path_msg.poses:
            dx = waypoint.pose.position.x - x
            dy = waypoint.pose.position.y - y

            distance = sqrt(dx**2 + dy**2)

            if distance <= 5.0:
                dot_product = heading_vector[0] * dx + heading_vector[1] * dy
                if dot_product > 0:
                    local_x = dx * cos(yaw) + dy * sin(yaw)
                    local_y = -dx * sin(yaw) + dy * cos(yaw)
                    local_points.append((local_x, local_y))

        # 로컬 좌표로 변환된 점들 중 가장 가까운 50개의 점 선택
        if len(local_points) >= 5:
            local_points.sort(key=lambda point: sqrt(point[0]**2 + point[1]**2))
            selected_points = local_points[:self.local_path_size]

            # x, y 좌표 리스트로 분리
            local_x_list = [point[0] for point in selected_points]
            local_y_list = [point[1] for point in selected_points]

            # 4차 다항식으로 근사
            coefficients = np.polyfit(local_x_list, local_y_list, 4)
            polynomial = np.poly1d(coefficients)

            # 근사된 다항식을 사용하여 로컬 경로 생성
            for i, local_x in enumerate(local_x_list):
                local_y = polynomial(local_x)
                rospy.loginfo(f"Fitted path point {i}: local_x={local_x}, local_y={local_y}")

        else:
            rospy.logwarn("Not enough points to fit polynomial.")

    def gps_callback(self, msg):
        # GPS에서 변환된 좌표를 받아서 현재 위치로 사용
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.is_status = True

    def global_path_callback(self, msg):
        # 글로벌 경로 콜백 함수: 경로 데이터를 받아 저장
        self.global_path_msg = msg

    def imu_callback(self, msg):
        # IMU의 orientation에서 yaw 값을 추출
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        # yaw 값 저장 (라디안 단위)
        self.yaw = yaw

if __name__ == '__main__':
    try:
        test_track = path_pub()
    except rospy.ROSInterruptException:
        pass
