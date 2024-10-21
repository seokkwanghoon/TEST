#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from stanley import EgoState, Stanley
from tf.transformations import euler_from_quaternion
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaSpeedometer
from sensor_msgs.msg import NavSatFix, Imu
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point, Twist
import math
import time
from pid import PID
from nav_msgs.msg import Odometry
class Drive:
    def __init__(self):
        self.x_vals = []
        self.y_vals = []
        self.heading = 0
        self.x = 0 
        self.y = 0
        self.cur_speed = 0
        self.prev_e = 0
        self.prev_n = 0
        self.prev_u = 0
        self.current_time = None

        #---------------------------acc variables----------------------------------#
        self.flag_acc = 0  # planning 단에서 flag 던져줌
        self.standstill_distance = 5
        self.lead_x = 0
        self.lead_y = 0
        self.lead_v = 0
        self.lead_heading = 0
        self.lead_distance = 0
        self.time_headway = 3
        self.target_distance = 0 
        self.pid_acc = PID(0.05,0.0025,0.00)
        #--------------------------------------------------------------------------#
        self.EARTH_RADIUS_EQUA = 6378137.0

        #--------------------------------------------------------------------------#
        self.stanley_controller = Stanley()
        self.ego = EgoState()
        self.pid_v = PID(0.05,0.0025,0.00)  # PID 인스턴스 생성
        rospy.init_node('tracking_node', anonymous=True)
        
        rospy.Subscriber('/enu_coordinates', Point, self.enu_callback)
        rospy.Subscriber('/carla/hero/IMU', Imu, self.imu_callback)
        rospy.Subscriber('/carla/hero/Speed', CarlaSpeedometer, self.speed_callback)
        rospy.Subscriber('/clock', Clock, self.clock_callback)
        rospy.Subscriber('/carla/lead_vehicle_status', Twist, self.lead_vehicle_callback)
        self.pub = rospy.Publisher('/carla/hero/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=5)
        self.control_msg = CarlaEgoVehicleControl()
        
        # Initial ENU reference point
        self.lat0 = 0.0
        self.lon0 = 0.0
        self.alt0 = 0.0
    
    #-------------------------callback functions start-------------------------------#
    def clock_callback(self, clock_msg):
        self.current_time = clock_msg.clock

    def enu_callback(self, data):
            self.x = data.x
            self.y = data.y
    
    def odom_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def imu_callback(self, data):
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        )
        euler_angles = euler_from_quaternion(quaternion)
        self.heading = euler_angles[2]
        self.heading = self.heading * 180 / np.pi

    def speed_callback(self, msg):
        self.cur_speed = msg.speed * 3.6  # m/s를 km/h로 변환
    
    def lead_vehicle_callback(self,data):
        self.lead_x = data.linear.x
        self.lead_y = data.linear.y
        self.lead_v = data.linear.z
        self.lead_heading = data.angular.z
    #-------------------------callback functions finish-------------------------------#

    def deg2rad(self,degrees):
        return  degrees * math.pi / 180.0

    def rad2deg(self,radians):
        return radians * 180.0 / math.pi

    def run(self):
        self.stanley_controller.set_ref_path('smoothing_path.csv')  # 경로 설정
        target_speed = int(input("목표 속도 설정 (km/h): "))
        rate = rospy.Rate(20)  # 20Hz로 루프 실행
        
        while not rospy.is_shutdown():
            self.ego.x = self.x
            self.ego.y = self.y
            print(f'x: {self.x}, y: {self.y}')
            self.ego.heading = self.heading
            self.ego.velocity = self.cur_speed  # 현재 속도를 업데이트

            self.stanley_controller.set_state(self.ego)
            self.flag_acc, self.lead_distance = self.stanley_controller.acc_flag(self.lead_x,self.lead_y,self.lead_v)
            steer = self.stanley_controller.main(self.cur_speed)
            
            # reference velocity tracking , acc tracking
            speed_error = target_speed - self.cur_speed
            self.target_distance = self.standstill_distance + self.time_headway * self.ego.velocity
            distance_error = self.target_distance - self.lead_distance
#----------------------------------------종방향 제어기 설정---------------------------------------#
            if self.flag_acc == 0:
                throttle = self.pid_v.compute(speed_error)
                throttle = max(0, min(throttle, 1))  # throttle 범위 제한 (0~1)

                if speed_error < 0:
                    self.control_msg.throttle = 0
                    self.control_msg.brake = max(0, min(-speed_error / target_speed, 1))  # 브레이크 범위 제한 (0~1)
                else:
                    self.control_msg.brake = 0
                    self.control_msg.throttle = throttle
            
                self.control_msg.steer = steer
                
                if self.current_time is not None:
                    self.control_msg.header.stamp = self.current_time
                    self.control_msg.header.frame_id = "base_link"

                self.pub.publish(self.control_msg)
                rate.sleep()  # 20 Hz로 루프 유지
                
            elif self.flag_acc == 1:
                throttle = self.pid_acc.compute(distance_error)
                throttle = max(0, min(throttle, 1))

                if distance_error < 0:
                    self.control_msg.brake = 0
                    self.control_msg.throttle = throttle
                else:
                    self.control_msg.throttle = 0
                    self.control_msg.brake = max(0, min(-distance_error / self.target_distance, 1))
                self.control_msg.steer = steer
                
                if self.current_time is not None:
                    self.control_msg.header.stamp = self.current_time
                    self.control_msg.header.frame_id = "base_link"

                self.pub.publish(self.control_msg)
                rate.sleep()  # 20 Hz로 루프 유지
#------------------------------------------------------------------------------------------------#
if __name__ == '__main__':
    drive = Drive()
    drive.run()
