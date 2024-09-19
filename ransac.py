#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import random

class Plane:
    def __init__(self):
        self.inliers = []
        self.equation = []

    def fit(self, pts, thresh=0.05, maxIteration=1000):
        n_points = pts.shape[0]
        best_eq = []
        best_inliers = []

        for it in range(maxIteration):
            # 3개의 랜덤 포인트 샘플링
            id_samples = random.sample(range(0, n_points), 3)
            pt_samples = pts[id_samples]

            # 평면을 정의하는 두 벡터 계산
            vecA = pt_samples[1, :] - pt_samples[0, :]
            vecB = pt_samples[2, :] - pt_samples[0, :]
            vecC = np.cross(vecA, vecB)
            vecC = vecC / np.linalg.norm(vecC)

            # 평면 방정식 계산
            k = -np.sum(np.multiply(vecC, pt_samples[0, :]))
            plane_eq = [vecC[0], vecC[1], vecC[2], k]

            # 포인트와 평면 간의 거리 계산
            dist_pt = (
                plane_eq[0] * pts[:, 0] + plane_eq[1] * pts[:, 1] +
                plane_eq[2] * pts[:, 2] + plane_eq[3]
            ) / np.sqrt(plane_eq[0] ** 2 + plane_eq[1] ** 2 + plane_eq[2] ** 2)

            # inlier 인덱스 선택
            pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]
            if len(pt_id_inliers) > len(best_inliers):
                best_eq = plane_eq
                best_inliers = pt_id_inliers

        self.inliers = best_inliers
        self.equation = best_eq
        return self.equation, self.inliers

def point_cloud_callback(msg):
    point_cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(point_cloud))
    
    if points.shape[0] < 3:
        rospy.logwarn("Not enough points for plane fitting.")
        return

    plane = Plane()
    equation, inliers = plane.fit(points)

    if equation:
        rospy.loginfo(f"Plane equation: {equation}, Inliers count: {len(inliers)}")
        
        # 지면 포인트 제거
        non_ground_points = np.delete(points, inliers, axis=0)
        rospy.loginfo(f"Remaining points after ground removal: {len(non_ground_points)}")
        
        # 추가 처리: non_ground_points를 여기서 사용할 수 있음
    else:
        rospy.loginfo("No plane found.")

def listener():
    rospy.init_node('velodyne_listener', anonymous=True)
    rospy.Subscriber("/velodyne_points", PointCloud2, point_cloud_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

