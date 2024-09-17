#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from rubber_cone_mission.msg import Fin  # 사용자 정의 메시지 타입
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point  # Point 메시지 타입 임포트 추가
import math
import numpy as np

PI = 3.14159265
thresh_dist = 0.5  # 두 점 사이의 거리 임계값
angle_limit_deg = 70  # 전방 ±30도, 즉 60도 범위

pub = None
marker_pub = None  # Marker 퍼블리셔

def cluster_callback(lsrscan_msg):
    global pub, marker_pub
    msg = Fin()

    num_of_clusters = 0
    num_of_points = []  # 여기에 클러스터별 포인트 수를 저장하는 리스트 초기화
    dist_points = []
    sign_distX = []
    sign_distY = []

    length_range = len(lsrscan_msg.ranges)
    rospy.loginfo("Length of Array = %d", length_range)

    prev_inf = True
    x_prev = y_prev = 0.0
    x_first = y_first = x_last = y_last = 0.0

    # 라이다 각도 범위 설정 (60도: ±30도)
    angle_min_deg = lsrscan_msg.angle_min * (180 / PI)
    angle_max_deg = lsrscan_msg.angle_max * (180 / PI)
    angle_increment_deg = lsrscan_msg.angle_increment * (180 / PI)

    # 60도 범위 내에서 인덱스 계산
    start_angle = -angle_limit_deg  # -30도
    end_angle = angle_limit_deg     # +30도

    # 시작 인덱스와 끝 인덱스 계산
    start_idx = int((start_angle - angle_min_deg) / angle_increment_deg)
    end_idx = int((end_angle - angle_min_deg) / angle_increment_deg)

    # 범위를 제한하여 처리
    for i in range(max(0, start_idx), min(length_range, end_idx)):
        distance_i = lsrscan_msg.ranges[i]
        if np.isfinite(distance_i):
            theta = lsrscan_msg.angle_min + (i * lsrscan_msg.angle_increment)
            x_curr = distance_i * math.cos(theta)
            y_curr = distance_i * math.sin(theta)

            if not prev_inf:
                distance = math.sqrt((x_curr - x_prev) ** 2 + (y_curr - y_prev) ** 2)
                if distance > thresh_dist:
                    num_of_clusters += 1
                    num_of_points.append(1)  # 새로운 클러스터에 첫 번째 포인트 추가
                else:
                    num_of_points[-1] += 1  # 같은 클러스터에 포인트 추가
            else:
                num_of_clusters += 1
                num_of_points.append(1)  # 새로운 클러스터 시작

            x_prev, y_prev = x_curr, y_curr
            prev_inf = False

            dist_points.append(distance_i)
            sign_distX.append(x_curr)
            sign_distY.append(y_curr)

            if i == 0:
                x_first, y_first = x_curr, y_curr
        else:
            prev_inf = True

    # 첫 번째와 마지막 포인트 연결
    x_last, y_last = x_prev, y_prev
    distance = math.sqrt((x_first - x_last) ** 2 + (y_first - y_last) ** 2)
    if distance <= thresh_dist and num_of_clusters > 1:
        num_of_points[0] += num_of_points[-1]  # 첫 번째와 마지막 클러스터 병합
        num_of_clusters -= 1
        num_of_points.pop(-1)

    rospy.loginfo("No. of Clusters : %d", num_of_clusters)

    sum_points = 0
    centroids_x = []
    centroids_y = []

    # 여기서 marker 객체를 생성해야 합니다.
    marker = Marker()
    marker.header.frame_id = "cloud"  # 적절한 프레임 ID 사용
    marker.header.stamp = rospy.Time.now()
    marker.ns = "centroids"
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.1  # 점 크기
    marker.scale.y = 0.1
    marker.color.r = 1.0  # 빨간색 점
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # 불투명

    # 클러스터 처리 부분에서 y좌표가 양수인 클러스터만 처리
    for idx, points_in_cluster in enumerate(num_of_points):
        start_idx = sum_points
        end_idx = sum_points + points_in_cluster

        cluster_x = sign_distX[start_idx:end_idx]
        cluster_y = sign_distY[start_idx:end_idx]
        cluster_ranges = dist_points[start_idx:end_idx]  # 클러스터 내 거리값

        min_distance = min(cluster_ranges)  # 클러스터 내 최소 거리

        # 조건 추가: 최소 거리 < 1m 그리고 포인트가 15개 이상일 때만 처리
        if min_distance < 1 and points_in_cluster > 15:
            # 왼쪽에 있는 클러스터만 처리 (y > 0 인 경우만)
            if np.mean(cluster_y) > 0:
                # 유효한 클러스터의 중심점(centroid) 계산
                centroid_x = np.mean(cluster_x)
                centroid_y = np.mean(cluster_y)

                rospy.loginfo(f"Cluster {idx + 1}: Centroid at x = {centroid_x}, y = {centroid_y}")

                # 중심점 좌표를 리스트에 추가
                centroids_x.append(centroid_x)
                centroids_y.append(centroid_y)

                # Marker에 centroid 추가
                point = Point()
                point.x = centroid_x
                point.y = centroid_y
                point.z = 0  # 2D 이므로 z는 0으로 설정

                marker.points.append(point)  # Point 객체를 marker에 추가

        sum_points += points_in_cluster

    # 유효한 클러스터가 있을 경우에만 퍼블리시
    if centroids_x:
        msg.num = len(centroids_x)
        msg.aveD = centroids_x  # X 좌표 리스트로 사용
        msg.minD = centroids_y  # Y 좌표 리스트로 사용
        pub.publish(msg)
        
        # RViz에 시각화를 위한 Marker 퍼블리시
        marker_pub.publish(marker)

def main():
    global pub, marker_pub
    rospy.init_node('centroid_publisher')

    # Fin 메시지 퍼블리셔
    pub = rospy.Publisher('fin_data', Fin, queue_size=10)
    
    # RViz 시각화를 위한 Marker 퍼블리셔
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # 라이다 데이터 구독
    rospy.Subscriber('/scan', LaserScan, cluster_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
