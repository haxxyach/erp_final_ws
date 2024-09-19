#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import threading
from math import sqrt, cos, sin
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


class PathPub:
    def __init__(self):
        rospy.init_node('path_pub', anonymous=True)

        # 토픽 구독자 설정
        rospy.Subscriber("/gps_map", Float32MultiArray, self.gps_callback)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)

        # 로컬 패스 발행자 설정
        self.local_path_pub = rospy.Publisher("/local_path", Path, queue_size=1)

        # 로컬 패스 생성 시 사용할 변수 초기화
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'
        self.is_status = False  # 경로 계산 가능 여부 플래그
        self.local_path_size = 50  # 로컬 경로 크기

        self.x = 0  # 현재 위치 x 좌표
        self.y = 0  # 현재 위치 y 좌표
        self.yaw = 0  # 차량의 헤딩 (yaw 값)

        # Lookahead Distance 설정 (변경하기 쉽게 설정)
        self.lookahead_distance = 3.0  # 3m으로 설정, 필요 시 쉽게 조정 가능
        self.lookahead_point = None  # Lookahead Point 좌표 저장용 변수

        # 그래프 표시를 위한 변수 초기화
        self.fig = None
        self.ax = None
        self.graph_initialized = False

        # 그래프를 표시하기 위한 스레드 시작
        self.graph_thread = threading.Thread(target=self.show_graph)
        self.graph_thread.start()

        rate = rospy.Rate(10)  # 루프 주기를 10Hz로 설정
        while not rospy.is_shutdown():
            if self.is_status:
                self.process_local_path()

            rate.sleep()

        # 노드가 종료되면 그래프 창도 닫기
        plt.close(self.fig)

    def show_graph(self):
        # 그래프 초기화
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('Polynomial Fit')
        self.ax.set_xlabel('Local Y (West)')
        self.ax.set_ylabel('Local X (North)')
        self.ax.set_xlim(-10, 10)  # x축 범위 고정
        self.ax.set_ylim(-10, 10)  # y축 범위 고정
        self.ax.axhline(0, color='gray', linestyle='--')
        self.ax.axvline(0, color='gray', linestyle='--')
        self.graph_initialized = True
        plt.show(block=True)

    def process_local_path(self):
        x = self.x
        y = self.y

        # 차량의 진행 방향 벡터 계산 (글로벌 좌표계 기준)
        heading_vector = (cos(self.yaw), sin(self.yaw))

        local_points = []

        # 글로벌 경로에서 로컬 좌표로 변환된 경로 점 선택
        for waypoint in self.global_path_msg.poses:
            dx = waypoint.pose.position.x - x
            dy = waypoint.pose.position.y - y

            distance = sqrt(dx**2 + dy**2)

            if distance <= 5.0:
                dot_product = heading_vector[0] * dx + heading_vector[1] * dy
                if dot_product > 0:
                    local_x = dx * cos(self.yaw) + dy * sin(self.yaw)
                    local_y = -dx * sin(self.yaw) + dy * cos(self.yaw)
                    local_points.append((local_x, local_y))

        # 로컬 좌표로 변환된 점들 중 가장 가까운 50개의 점 선택
        if len(local_points) >= 5:
            self.fit_and_publish_path(local_points)

    def fit_and_publish_path(self, local_points):
        local_x_list = np.array([point[0] for point in local_points[:self.local_path_size]])
        local_y_list = np.array([point[1] for point in local_points[:self.local_path_size]])

        coefficients = np.polyfit(local_x_list, local_y_list, 4)
        polynomial = np.poly1d(coefficients)

        local_path_msg = Path()
        local_path_msg.header.frame_id = 'map'

        for local_x in local_x_list:
            local_y = polynomial(local_x)
            pose = PoseStamped()
            pose.pose.position.x = local_x
            pose.pose.position.y = local_y
            pose.pose.position.z = 0  # z값은 0으로 설정
            local_path_msg.poses.append(pose)

        if len(local_path_msg.poses) > 0:
            self.local_path_pub.publish(local_path_msg)

        self.update_graph(local_x_list, local_y_list, polynomial)

    def update_graph(self, local_x_list, local_y_list, polynomial):
        if self.graph_initialized:
            try:
                # x_new와 y_new 계산 (x축이 북쪽을 향하도록 x와 y를 교환)
                x_new = np.linspace(min(local_x_list), max(local_x_list), 100)
                y_new = polynomial(x_new)

                self.ax.clear()

                # 피팅에 사용된 점들 (파란색으로 표시)
                self.ax.plot(-local_y_list, local_x_list, 'bo', label='Data Points')

                # 피팅된 다항식에 의해 계산된 점들 (빨간색으로 표시)
                self.ax.plot(-y_new, x_new, 'ro', label='Fitted Points')

                # 피팅된 다항식 곡선
                self.ax.plot(-y_new, x_new, 'r-', label='Fitted Polynomial')

                # Lookahead Point가 있으면 노란 점으로 표시
                if self.lookahead_point is not None:
                    lookahead_x, lookahead_y = self.lookahead_point
                    self.ax.plot(-lookahead_y, lookahead_x, 'yo', label='Lookahead Point')

                # x축과 y축에 점선 표시
                self.ax.axhline(0, color='gray', linestyle='--')
                self.ax.axvline(0, color='gray', linestyle='--')

                # (0,0)에 빨간 점 표시 (차량 위치)
                self.ax.plot(0, 0, 'ro', label='Vehicle Position')

                self.ax.set_title('Polynomial Fit')
                self.ax.set_xlabel('Local Y (West)')
                self.ax.set_ylabel('Local X (North)')
                self.ax.legend()
                self.ax.set_aspect('equal', adjustable='box')

                # 축 범위 설정 (고정)
                self.ax.set_xlim(-10, 10)
                self.ax.set_ylim(-10, 10)

                # 업데이트 시 화면을 다시 그리도록 수정
                self.fig.canvas.draw_idle()

            except IndexError as e:
                rospy.logwarn(f"Graph update encountered an error: {e}")
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")

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
        test_track = PathPub()
    except rospy.ROSInterruptException:
        pass
