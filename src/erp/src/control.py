#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os
import rospkg
import math
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from sensor_msgs.msg import Imu


class Lateral_Control :
    def __init__(self):
        
        rospy.init_node('control', anonymous=True)
        
        rospy.Subscriber("local_path", Path, self.local_path_callback)
        rospy.Subscriber("Ego_topic", EgoVehicleStatus, self.status_callback)
        # rospy.Subscriber("/imu", Imu, self.imu_callback)
        
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd_0', CtrlCmd, queue_size=1)
        
        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=2

        self.is_path=False
        
        self.local_x = None
        self.local_y = None

        self.vehicle_length=1.63
        
        # Lookahead Distance 설정
        self.lookahead_distance = 3.0

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():

            if self.is_path ==True:

                self.ctrl_cmd_msg.steering = math.atan2((2*self.vehicle_length)*self.local_y,self.lookahead_distance**2)
                self.ctrl_cmd_msg.velocity = 20.0 # 원하는 고정 속도 값 입력해
                    

                os.system('clear')
                print("-------------------------------------")
                print(" steering (deg) = ", self.ctrl_cmd_msg.steering * 180/math.pi)
                print(" velocity (km/h) = ", self.ctrl_cmd_msg.velocity)
                print("-------------------------------------")
                
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg) # 최종 제어 명령 퍼블리시

            else:
                os.system('clear')
                if not self.is_path:
                    print("[1] can't subscribe '/local_path' topic...")
            
            self.is_path = False
            rate.sleep()

    def local_path_callback(self,msg):
        
        self.is_path=True
        self.local_x = msg.poses[0].pose.position.x
        self.local_y = msg.poses[0].pose.position.y  
        
        print("Updated local_x = %.2f, local_y = %.2f", self.local_x, self.local_y)
    
    
    # def status_callback(self, msg):
    #     self.is_status = True
    #     self.current_vel = msg.velocity.x
    
    #def imu_callback(self,msg):
    
# class PID_Control:
#     def __init__(self):

#         self.prev_error = 0
#         self.i_control = 0
#         self.controlTime = 0.02
        
#         self.kp = 0.1
#         self.ki = 0.05
#         self.kd = 0.01

#     def pid(self,vel_d, vel):
#         error = vel_d - vel
#         p_control = self.kp * error
#         if error <= 5:
#             self.i_control += self.ki * error * self.controlTime
#         d_control = self.kd * (error-self.prev_error) / self.controlTime
#         self.prev_error = error
        
#         return p_control + self.i_control + d_control


if __name__ == '__main__':
    try:
        lateralctrl=Lateral_Control()
    except rospy.ROSInterruptException:
        pass
