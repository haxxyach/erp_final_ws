#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64  # 조향각 데이터 타입
from rubber_cone_mission.msg import Fin  # 클러스터 centroid 데이터를 받기 위한 사용자 정의 메시지 타입
from geometry_msgs.msg import Point  # 웨이포인트 좌표 퍼블리시를 위한 메시지 타입
from visualization_msgs.msg import Marker  # RViz 시각화를 위한 Marker 메시지

# 전역 변수
current_steering_angle = 0.0  # 초기 조향각 값
current_centroids_x = []  # 초기 클러스터 중심점 X 좌표 리스트
current_centroids_y = []  # 초기 클러스터 중심점 Y 좌표 리스트

def steering_angle_callback(msg):
    """차량의 조향각을 구독하는 콜백 함수"""
    global current_steering_angle
    current_steering_angle = msg.data  # 조향각 업데이트
    rospy.loginfo(f"Received steering angle: {current_steering_angle}")

def centroids_callback(msg):
    """클러스터의 중심점(centroid) 데이터를 구독하는 콜백 함수"""
    global current_centroids_x, current_centroids_y
    current_centroids_x = msg.aveD  # X 좌표 리스트
    current_centroids_y = msg.minD  # Y 좌표 리스트
    rospy.loginfo(f"Received {len(current_centroids_x)} centroids.")

def calculate_offset_from_steering(steering_angle):
    """조향각을 기반으로 동적 오프셋 계산"""
    base_offset = 1.0  # 직진 시 적용할 기본 오프셋

    # 좌회전(음수 조향각): 오프셋을 더 크게 설정
    # 우회전(양수 조향각): 오프셋을 더 작게 설정
    if steering_angle < 0:  # 좌회전
        dynamic_offset = base_offset * (1 + abs(steering_angle) / 30)  # 조향각 30도 기준
    else:  # 우회전
        dynamic_offset = base_offset * (1 - abs(steering_angle) / 30)

    # 오프셋이 너무 작아지지 않도록 최소값 설정
    dynamic_offset = max(dynamic_offset, 0.5)  # 최소 오프셋 값
    return dynamic_offset

def calculate_waypoint(steering_angle, centroids_x, centroids_y):
    """조향각과 클러스터 중심점 데이터를 바탕으로 웨이포인트 계산"""
    if not centroids_x or not centroids_y:
        rospy.logwarn("No centroids available to calculate waypoint.")
        return None

    # 현재는 간단히 가장 가까운 클러스터 중심점을 웨이포인트로 설정
    closest_centroid_index = 0  # 첫 번째 클러스터가 가장 가까운 클러스터로 가정
    closest_centroid_x = centroids_x[closest_centroid_index]
    closest_centroid_y = centroids_y[closest_centroid_index]

    # 조향각에 따른 동적 오프셋 적용
    offset = calculate_offset_from_steering(steering_angle)
    waypoint_x = closest_centroid_x
    waypoint_y = closest_centroid_y - offset  # y축 방향으로 오프셋 적용

    rospy.loginfo(f"Calculated waypoint: x = {waypoint_x}, y = {waypoint_y}")
    return waypoint_x, waypoint_y

def main():
    rospy.init_node('waypoint_calculator_node')

    # 구독: 조향각 및 클러스터 중심점
    rospy.Subscriber('/steering_angle', Float64, steering_angle_callback)  # 조향각 토픽
    rospy.Subscriber('/fin_data', Fin, centroids_callback)  # 클러스터 중심점 토픽

    # 퍼블리시: 웨이포인트 좌표
    waypoint_pub = rospy.Publisher('/waypoint', Point, queue_size=10)

    # 퍼블리시: 웨이포인트 시각화를 위한 Marker
    waypoint_marker_pub = rospy.Publisher('/waypoint_marker', Marker, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz 주기로 웨이포인트 계산 및 퍼블리시
    while not rospy.is_shutdown():
        # 웨이포인트 계산
        waypoint = calculate_waypoint(current_steering_angle, current_centroids_x, current_centroids_y)
        if waypoint:
            # 웨이포인트를 퍼블리시
            waypoint_msg = Point()
            waypoint_msg.x = waypoint[0]
            waypoint_msg.y = waypoint[1]
            waypoint_msg.z = 0.0  # 2D이므로 z는 0으로 설정
            waypoint_pub.publish(waypoint_msg)

            # Marker 생성 및 퍼블리시
            marker = Marker()
            marker.header.frame_id = "cloud"  # 라이다의 프레임 ID로 설정
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoint_marker"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0

            marker.scale.x = 0.2  # Marker 크기 설정
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = 0.0  # 파란색으로 설정
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0  # 투명도

            waypoint_marker_pub.publish(marker)  # Marker 퍼블리시

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
