#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Point
from erp_driver.msg import erpCmdMsg

# ERP42 제어 명령 퍼블리셔
erp_cmd_pub = None

# 웨이포인트 콜백 함수
def waypoint_callback(waypoint):
    global erp_cmd_pub

    # 웨이포인트의 x, y 좌표
    waypoint_x = waypoint.x
    waypoint_y = waypoint.y

    # ERP42가 웨이포인트를 향해 갈 수 있도록 거리 및 조향각 계산
    distance = math.sqrt(waypoint_x ** 2 + waypoint_y ** 2)
    steering_angle = math.atan2(waypoint_y, waypoint_x) * (180.0 / math.pi)  # 라디안을 도 단위로 변환

    # 제어 명령 메시지 생성
    cmd_msg = erpCmdMsg()

    # 속도 설정 (거리에 따라 다르게 설정할 수 있음)
    if distance > 1.0:  # 웨이포인트가 1m 이상 떨어져 있으면
        cmd_msg.speed = 50  # 속도 50
    else:
        cmd_msg.speed = 30  # 웨이포인트와 가까워질수록 속도 감소

    # 조향각 설정
    cmd_msg.steer = int(steering_angle)

    # 기어 및 기타 설정
    cmd_msg.gear = 0  # 전진
    cmd_msg.brake = 1 if distance < 0.2 else 0  # 웨이포인트와 매우 가까워지면 브레이크

    # 제어 명령 퍼블리시
    erp_cmd_pub.publish(cmd_msg)

    rospy.loginfo(f"Steering angle: {cmd_msg.steer}, Speed: {cmd_msg.speed}")

# 메인 함수
def erp42_controller():
    global erp_cmd_pub

    # ROS 노드 초기화
    rospy.init_node('erp42_waypoint_controller', anonymous=True)

    # ERP42 제어 명령 퍼블리셔
    erp_cmd_pub = rospy.Publisher('/erp42_ctrl_cmd', erpCmdMsg, queue_size=10)

    # 웨이포인트 토픽 구독
    rospy.Subscriber('/waypoint', Point, waypoint_callback)

    rospy.loginfo("ERP42 waypoint controller started.")

    # ROS 스핀
    rospy.spin()

if __name__ == '__main__':
    try:
        erp42_controller()
    except rospy.ROSInterruptException:
        pass
