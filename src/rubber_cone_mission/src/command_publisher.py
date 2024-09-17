#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point  # 웨이포인트 메시지 타입
from std_msgs.msg import Float64  # 조향과 속도를 위한 메시지 타입
import math

# 파라미터
CONSTANT_SPEED = 10  # 상수 속도 (단위: km/h)
MAX_STEERING_ANGLE = 30.0  # 최대 조향각 (단위: degrees)

def waypoint_callback(msg):
    """웨이포인트를 구독하고 조향각과 속도를 계산하는 콜백 함수"""
    global pub_steering, pub_speed

    # 현재 차량의 위치 (예: (0, 0)을 차량 기준으로 가정)
    current_x = 0.0
    current_y = 0.0

    # 웨이포인트 좌표
    waypoint_x = msg.x
    waypoint_y = msg.y

    # 웨이포인트까지의 각도 계산 (atan2 함수 사용)
    angle_to_waypoint = math.degrees(math.atan2(waypoint_y - current_y, waypoint_x - current_x))

    # 각도를 조향각으로 변환 (일정 범위로 제한)
    steering_angle = max(min(angle_to_waypoint, MAX_STEERING_ANGLE), -MAX_STEERING_ANGLE)

    # 웨이포인트까지의 거리 계산
    distance_to_waypoint = math.sqrt((waypoint_x - current_x)**2 + (waypoint_y - current_y)**2)

    # 속도를 상수로 설정
    speed = CONSTANT_SPEED

    # 거리 기반으로 속도 설정
    # if distance_to_waypoint < 1.0:
    #     speed = MIN_SPEED
    # else:
    #     speed = MAX_SPEED

    # 조향각과 속도를 퍼블리시
    pub_steering.publish(steering_angle)
    pub_speed.publish(speed)

    rospy.loginfo(f"Steering: {steering_angle}, Speed: {speed}")

def main():
    global pub_steering, pub_speed
    rospy.init_node('command_publisher', anonymous=True)

    # 웨이포인트 구독
    rospy.Subscriber('/waypoint', Point, waypoint_callback)

    # 조향과 속도 퍼블리시
    pub_steering = rospy.Publisher('/steering_angle_cmd', Float64, queue_size=10)
    pub_speed = rospy.Publisher('/speed_cmd', Float64, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass