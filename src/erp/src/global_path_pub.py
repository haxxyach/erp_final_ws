#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, atan2
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

class read_path_pub:

    def __init__(self):
        rospy.init_node('read_path_pub', anonymous=True)
        
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.road_marker_pub = rospy.Publisher('/road_marker', Marker, queue_size=1)

        # global_path_msg 초기화
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = 'map'

        # road_marker 초기화
        self.road_marker = Marker()
        self.road_marker.header.frame_id = 'map'
        self.road_marker.type = Marker.TRIANGLE_LIST
        self.road_marker.action = Marker.ADD
        self.road_marker.scale.x = 1.0
        self.road_marker.scale.y = 1.0
        self.road_marker.scale.z = 1.0
        self.road_marker.color.a = 1.0
        self.road_marker.color.r = 0.5
        self.road_marker.color.g = 0.5
        self.road_marker.color.b = 0.5

        full_path = '/home/unita/erp_final_ws/src/erp/src/test.txt'
        with open(full_path, 'r') as f:
            lines = f.readlines()

        # 파일에서 경로 읽어오기
        path_points = []
        for line in lines:
            tmp = line.split()
            x = float(tmp[0])
            y = float(tmp[1])
            path_points.append((x, y))
            read_pose = PoseStamped()
            read_pose.pose.position.x = x
            read_pose.pose.position.y = y
            read_pose.pose.orientation.w = 1
            self.global_path_msg.poses.append(read_pose)
        
        # 좌우 경계 계산
        left_points = []
        right_points = []
        width = 0.7  # 도로의 절반 폭

        for i in range(len(path_points)):
            x = path_points[i][0]
            y = path_points[i][1]

            # 방향 계산
            if i < len(path_points) - 1:
                dx = path_points[i+1][0] - x
                dy = path_points[i+1][1] - y
            else:
                dx = x - path_points[i-1][0]
                dy = y - path_points[i-1][1]
            heading = atan2(dy, dx)

            # 좌우 오프셋 계산
            left_x = x - width * sin(heading)
            left_y = y + width * cos(heading)
            right_x = x + width * sin(heading)
            right_y = y - width * cos(heading)

            left_points.append((left_x, left_y))
            right_points.append((right_x, right_y))

        # 도로 영역 생성 (삼각형 리스트)
        for i in range(len(path_points) - 1):
            # 첫 번째 삼각형
            p1 = Point()
            p1.x, p1.y = left_points[i]
            p2 = Point()
            p2.x, p2.y = right_points[i]
            p3 = Point()
            p3.x, p3.y = left_points[i+1]

            self.road_marker.points.extend([p1, p2, p3])

            # 두 번째 삼각형
            p1 = Point()
            p1.x, p1.y = left_points[i+1]
            p2 = Point()
            p2.x, p2.y = right_points[i]
            p3 = Point()
            p3.x, p3.y = right_points[i+1]

            self.road_marker.points.extend([p1, p2, p3])

        # 경로에 있는 포인트 개수 출력
        rospy.loginfo("Number of points in the path: %d", len(self.global_path_msg.poses))

        rate = rospy.Rate(20)  # 20Hz
        while not rospy.is_shutdown():
            self.global_path_msg.header.stamp = rospy.Time.now()
            self.global_path_pub.publish(self.global_path_msg)
            self.road_marker.header.stamp = rospy.Time.now()
            self.road_marker_pub.publish(self.road_marker)
            rate.sleep()

if __name__ == '__main__':
    try:
        test_track = read_path_pub()
    except rospy.ROSInterruptException:
        pass