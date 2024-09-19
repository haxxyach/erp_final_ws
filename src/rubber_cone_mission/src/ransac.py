#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import random

class Plane:
    def __init__(self):
        self.inliers = []
        self.equation = []

    def fit(self, pts, thresh=0.01, maxIteration=1000):
        n_points = pts.shape[0]
        best_eq = []
        best_inliers = []

        for it in range(maxIteration):
            id_samples = random.sample(range(0, n_points), 3)
            pt_samples = pts[id_samples]

            vecA = pt_samples[1, :] - pt_samples[0, :]
            vecB = pt_samples[2, :] - pt_samples[0, :]
            vecC = np.cross(vecA, vecB)
            vecC = vecC / np.linalg.norm(vecC)

            k = -np.sum(np.multiply(vecC, pt_samples[0, :]))
            plane_eq = [vecC[0], vecC[1], vecC[2], k]

            dist_pt = (
                plane_eq[0] * pts[:, 0] + plane_eq[1] * pts[:, 1] +
                plane_eq[2] * pts[:, 2] + plane_eq[3]
            ) / np.sqrt(plane_eq[0] ** 2 + plane_eq[1] ** 2 + plane_eq[2] ** 2)

            pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]
            if len(pt_id_inliers) > len(best_inliers):
                best_eq = plane_eq
                best_inliers = pt_id_inliers

        self.inliers = best_inliers
        self.equation = best_eq
        return self.equation, self.inliers

class PointCloudProcessor:
    def __init__(self):
        self.pub = rospy.Publisher("/filtered_points", PointCloud2, queue_size=1)  # 필터링된 포인트 퍼블리시
        
    def apply_crop_filter(self, points):
        # 크롭 필터 적용 (특정 3D 영역 내의 포인트만 남김)
        x_min, x_max = 0, 3   # X 축 크롭 범위
        y_min, y_max = -3, 3     # Y 축 크롭 범위
        z_min, z_max = 0, 2     # Z 축 크롭 범위
        
        # 3D 영역 내에 있는 포인트들만 선택
        cropped_points = points[
            (points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
            (points[:, 1] >= y_min) & (points[:, 1] <= y_max) &
            (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        ]
        return cropped_points

    def point_cloud_callback(self, msg):
        # PointCloud2 메시지를 numpy 배열로 변환
        point_cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(point_cloud))
        
        if points.shape[0] < 3:
            rospy.logwarn("Not enough points for plane fitting.")
            return

        # 크롭 필터 적용
        cropped_points = self.apply_crop_filter(points)
        rospy.loginfo(f"Remaining points after crop filter: {len(cropped_points)}")

        # RANSAC 기반 평면 추정
        plane = Plane()
        equation, inliers = plane.fit(cropped_points)

        if equation:
            rospy.loginfo(f"Plane equation: {equation}, Inliers count: {len(inliers)}")
            
            # 지면에 해당하는 포인트들 제거
            non_ground_points = np.delete(cropped_points, inliers, axis=0)
            rospy.loginfo(f"Remaining points after ground removal: {len(non_ground_points)}")
            
            # 남은 포인트를 PointCloud2 형식으로 변환
            non_ground_cloud = pc2.create_cloud_xyz32(msg.header, non_ground_points)
            
            # 필터링된 포인트 클라우드를 퍼블리시
            self.pub.publish(non_ground_cloud)
        else:
            rospy.loginfo("No plane found.")

def listener():
    rospy.init_node('velodyne_listener', anonymous=True)
    processor = PointCloudProcessor()
    
    # Velodyne 포인트 클라우드 구독
    rospy.Subscriber("/velodyne_points", PointCloud2, processor.point_cloud_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
