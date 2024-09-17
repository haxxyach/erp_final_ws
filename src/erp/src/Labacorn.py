#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sklearn.cluster import DBSCAN
import numpy as np
import math

# 최소 및 최대 감지 거리
MIN_DIST = 0.2  # 20 cm
MAX_DIST = 2.5  # 2.5 meters

# Publisher for waypoints and markers
waypoint_pub = None
waypoint_marker_pub = None  # Waypoint marker publisher

# Marker 퍼블리셔
marker_pub = None

# 이전 마커 ID 저장을 위한 세트
prev_marker_ids = set()

def lidar_callback(scan_data):
    ranges = scan_data.ranges
    left_points = []
    right_points = []

    # 좌우 각도 범위 설정
    LEFT_MIN_ANGLE = math.radians(25)     # +10도
    LEFT_MAX_ANGLE = math.radians(90) # LiDAR의 최대 각도

    RIGHT_MIN_ANGLE = -math.radians(90)   # LiDAR의 최소 각도
    RIGHT_MAX_ANGLE = -math.radians(25)    # -10도

    # 각 포인트에 대해 각도와 거리를 계산하고, 좌우로 분류
    for i, distance in enumerate(ranges):
        if MIN_DIST < distance < MAX_DIST:
            angle = scan_data.angle_min + i * scan_data.angle_increment

            # 좌측 포인트 수집
            if LEFT_MIN_ANGLE <= angle <= LEFT_MAX_ANGLE:
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                left_points.append([x, y])

            # 우측 포인트 수집
            elif RIGHT_MIN_ANGLE <= angle <= RIGHT_MAX_ANGLE:
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                right_points.append([x, y])

    # 좌우 포인트를 합쳐서 전체 포인트로 만듦
    detected_points = left_points + right_points

    # DBSCAN 군집화 적용
    if detected_points:
        points = np.array(detected_points)
        labels = cluster_points(points)

        # 군집화된 객체 시각화 (마커 ID 관리 개선)
        publish_markers(points, labels)

        # 가장 가까운 왼쪽/오른쪽 객체 찾기
        create_closest_waypoint(points, labels)

def cluster_points(points):
    """주어진 포인트들에 대해 DBSCAN 클러스터링을 수행하고 레이블을 반환"""
    clustering = DBSCAN(eps=0.1, min_samples=3).fit(points)
    labels = clustering.labels_
    return labels

def publish_markers(points, labels):
    global prev_marker_ids  # 이전 마커 ID 세트
    marker_array = MarkerArray()
    unique_labels = set(labels)

    current_marker_ids = set()  # 현재 프레임에서 사용된 마커 ID

    for label in unique_labels:
        if label == -1:  # -1은 노이즈로 취급됨
            continue

        cluster_points = points[labels == label]
        mean_x = np.mean(cluster_points[:, 0])
        mean_y = np.mean(cluster_points[:, 1])

        marker = Marker()
        marker.header.frame_id = "base_link"  # 프레임 ID를 'base_link'로 설정
        marker.header.stamp = rospy.Time.now()

        marker.ns = "object_detection"
        marker.id = label  # 클러스터 레이블을 마커 ID로 사용
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # 마커의 위치 설정
        marker.pose.position.x = mean_x
        marker.pose.position.y = mean_y
        marker.pose.position.z = 0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # 좌우 판단하여 색상 설정
        if mean_y > 0:
            # y > 0이면 왼쪽이므로 파란색
            marker.color.a = 1.0  # 투명도
            marker.color.r = 0.0  # 빨간색 없음
            marker.color.g = 0.0  # 녹색 없음
            marker.color.b = 1.0  # 파란색
        else:
            # y <= 0이면 오른쪽이므로 빨간색
            marker.color.a = 1.0  # 투명도
            marker.color.r = 1.0  # 빨간색
            marker.color.g = 0.0  # 녹색 없음
            marker.color.b = 0.0  # 파란색 없음

        marker_array.markers.append(marker)
        current_marker_ids.add(label)  # 현재 마커 ID 추가

    # 이전 프레임의 마커 중 현재 프레임에 없는 마커 삭제
    markers_to_delete = prev_marker_ids - current_marker_ids
    for marker_id in markers_to_delete:
        marker = Marker()
        marker.header.frame_id = "base_link"  # 프레임 ID를 'base_link'로 설정
        marker.header.stamp = rospy.Time.now()
        marker.ns = "object_detection"
        marker.id = marker_id
        marker.action = Marker.DELETE
        marker_array.markers.append(marker)

    # 이전 마커 ID 업데이트
    prev_marker_ids = current_marker_ids

    marker_pub.publish(marker_array)

def create_closest_waypoint(points, labels):
    """가장 가까운 왼쪽/오른쪽 객체를 찾고, 그 사이의 중간 지점에 웨이포인트 생성"""
    unique_labels = set(labels)
    closest_left = None
    closest_right = None
    min_left_dist = float('inf')
    min_right_dist = float('inf')

    # 각 군집의 중앙을 찾고, 가장 가까운 왼쪽과 오른쪽 객체 찾기
    for label in unique_labels:
        if label == -1:  # 노이즈는 무시
            continue

        cluster_points = points[labels == label]
        mean_x = np.mean(cluster_points[:, 0])
        mean_y = np.mean(cluster_points[:, 1])
        distance = np.linalg.norm([mean_x, mean_y])  # LiDAR로부터의 거리 계산

        if mean_y > 0 and distance < min_left_dist:  # y > 0: 왼쪽
            closest_left = (mean_x, mean_y)
            min_left_dist = distance
        elif mean_y <= 0 and distance < min_right_dist:  # y <= 0: 오른쪽
            closest_right = (mean_x, mean_y)
            min_right_dist = distance

    # 왼쪽과 오른쪽 가장 가까운 점이 있을 때, 중간 지점에 웨이포인트 생성
    if closest_left and closest_right:
        left_x, left_y = closest_left
        right_x, right_y = closest_right

        # 중간 지점 계산
        waypoint_x = (left_x + right_x) / 2.0
        waypoint_y = (left_y + right_y) / 2.0

        # 웨이포인트 퍼블리시
        waypoint = Point()
        waypoint.x = waypoint_x
        waypoint.y = waypoint_y
        waypoint.z = 0.0

        waypoint_pub.publish(waypoint)

        # 웨이포인트 시각화
        visualize_waypoint(waypoint_x, waypoint_y)

        rospy.loginfo(f"Published waypoint: ({waypoint_x}, {waypoint_y})")

def visualize_waypoint(x, y):
    """웨이포인트를 시각화하기 위한 Marker 퍼블리시 함수"""
    marker = Marker()
    marker.header.frame_id = "base_link"  # 프레임 ID를 'base_link'로 설정
    marker.header.stamp = rospy.Time.now()

    marker.ns = "waypoint_visualization"
    marker.id = 0  # 웨이포인트는 하나씩만 시각화하므로 ID는 0으로 고정
    marker.type = Marker.CUBE  # 웨이포인트를 시각화할 마커 타입 (큐브 사용)
    marker.action = Marker.ADD

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.2  # 웨이포인트 마커 크기
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    marker.color.a = 1.0  # 투명도
    marker.color.r = 0.0  # 빨간색 없음
    marker.color.g = 1.0  # 녹색 (웨이포인트는 녹색으로 표시)
    marker.color.b = 0.0  # 파란색 없음

    # 웨이포인트 마커 퍼블리싱
    waypoint_marker_pub.publish(marker)

def lidar_object_detection():
    global marker_pub, waypoint_pub, waypoint_marker_pub

    rospy.init_node('lidar_object_detection_node', anonymous=True)

    # Marker 퍼블리셔
    marker_pub = rospy.Publisher('/detected_objects', MarkerArray, queue_size=10)

    # Waypoint 퍼블리셔
    waypoint_pub = rospy.Publisher('/waypoint', Point, queue_size=10)

    # Waypoint Marker 퍼블리셔 (시각화를 위한 퍼블리셔)
    waypoint_marker_pub = rospy.Publisher('/waypoint_marker', Marker, queue_size=10)

    # LaserScan 토픽 구독
    rospy.Subscriber('/scan', LaserScan, lidar_callback)

    rospy.loginfo("LiDAR object detection with closest waypoint generation started.")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        lidar_object_detection()
    except rospy.ROSInterruptException:
        pass
