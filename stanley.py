#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import numpy as np
from math import cos, sin, pi, sqrt, atan2, hypot, tan
import csv
import os
import rospkg
import math

class EgoState():
    velocity = 0.0
    x = 0.0
    y = 0.0
    heading = 0.0
    ax = 0.0

class Stanley():
    def __init__(self, dt=1/20):
        self.dt = dt
        self.ref_path = []
        # self.ref_path_xy = []
        self.path_num = 0
        self.newt_waypoint = []
        self.cur_waypoint = []
        self.closest_idx = 0
        self.idx = 0
        self.vehicle_length = 4.635
        self.k = 7  # 기본 Stanley 상수
        self.k_min = 7  # 최소 Stanley 상수
        self.k_max = 15  # 최대 Stanley 상수
        self.curvature_threshold = 0.0257  # 곡률에 따른 임계값
        self.ego = EgoState()

    def get_velocity(self, vel):
        self.velocity = vel

    def cal_HE(self):
        dx = self.next_waypoint[0] - self.cur_waypoint[0]
        dy = self.next_waypoint[1] - self.cur_waypoint[1]
        path_heading = atan2(dy, dx)  # radian
        heading_error = path_heading - self.yaw * pi / 180  # radian
        heading_error = (heading_error + pi) % (2 * pi) - pi
        return heading_error

    def cal_CTE(self):
        dx = self.next_waypoint[0] - self.cur_waypoint[0]
        dy = self.next_waypoint[1] - self.cur_waypoint[1]
        a = dy
        b = -dx
        c = dx * self.cur_waypoint[1] - dy * self.cur_waypoint[0]
        cte = (a * self.x + b * self.y + c) / sqrt(a ** 2 + b ** 2)
        return cte
    
    def set_state(self, ego):
        self.x = ego.x
        self.y = ego.y
        self.yaw = ego.heading
        self.velocity = ego.velocity

    # def find_min_idx(self, vehicle_position):
    #     # ref_path의 처음 두 열을 사용하여 거리 계산
    #     path_positions = self.ref_path[:, :2]
    #     distances = np.linalg.norm(path_positions - vehicle_position, axis=1)
    #     self.closest_idx = np.argmin(distances)
    #     return self.closest_idx

    def find_min_idx(self, vehicle_position):
        if self.closest_idx == 0:
            # 알고리즘이 처음 시작되었거나 재시작된 경우, 전체 경로를 검색
            start_idx = 0
            end_idx = len(self.ref_path)
        else:
            # 전방 300개의 포인트만 고려하여 거리 계산
            start_idx = self.closest_idx
            end_idx = min(self.closest_idx + 300, len(self.ref_path))

        # 고려할 경로 포인트들
        path_positions = self.ref_path[start_idx:end_idx, :2]
        distances = np.linalg.norm(path_positions - vehicle_position, axis=1)
        
        # 가장 가까운 인덱스를 찾음
        closest_local_idx = np.argmin(distances)
        best_idx = closest_local_idx + start_idx
        
        # 선택된 인덱스와 차량 간의 거리가 지나치게 멀지 않은지 확인
        if distances[closest_local_idx] > 10.0:  # 10m 이상 멀면 무시
            # 만약 처음 시작 시 경로가 너무 멀다면 closest_idx를 None으로 유지
            if self.closest_idx is None:
                return None
            else:
                return self.closest_idx
        
        # 인덱스를 업데이트
        self.closest_idx = best_idx
        return self.closest_idx

    def acc_flag(self,lead_x,lead_y,lead_v):
        x = self.x - lead_x
        y = self.y - lead_y
        lead_distance = math.sqrt(x**2 + y**2)
        if lead_distance > 35:
            flag = 0
        else:
            flag = 1
        return flag, lead_distance



    def set_ref_path(self, filename):
        rospack = rospkg.RosPack()
        filepath = os.path.join(rospack.get_path('carla_ros'), f'path/{filename}')
        with open(filepath, newline='') as csvfile:
            path_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
            self.ref_path = [list(map(float, row)) for row in path_reader]
        self.ref_path = np.array(self.ref_path)
        

    def calculate_curvature(self):
        step = 5
        if self.idx < step or self.idx >= len(self.ref_path) - step:
            return 0.0

        prev_point = self.ref_path[self.idx - step]
        current_point = self.ref_path[self.idx]
        next_point = self.ref_path[self.idx + step]

        # Calculate the curvature using the three points
        a = np.linalg.norm(current_point - prev_point)
        b = np.linalg.norm(next_point - current_point)
        c = np.linalg.norm(next_point - prev_point)

        if a == 0 or b == 0 or c == 0:
            return 0.0

        s = (a + b + c) / 2
        area = sqrt(max(0,s * (s - a) * (s - b) * (s - c)))

        if area == 0:
            return 0.0

        curvature = (4 * area) / (a * b * c)
        return curvature
    
    def adjust_k_based_on_curvature(self, curvature):
        if curvature > self.curvature_threshold:
            self.k = self.k_max
        else:
            self.k = self.k_min + (self.k_max - self.k_min) * curvature / self.curvature_threshold
        self.k = max(self.k_min, min(self.k, self.k_max))
        print(f"Adjusted k: {self.k}")

    def main(self, velocity):
        self.velocity = velocity
        # self.ref_path_xy = self.ref_path[:, 1:3]

        self.idx = self.find_min_idx([self.x, self.y])
        print(f'idx : {self.idx} , ego_speed : {self.velocity}')
        if self.idx < len(self.ref_path) - 1:
            self.cur_waypoint = self.ref_path[self.idx, :]
            self.next_waypoint = self.ref_path[self.idx + 1, :]
        else:
            self.cur_waypoint = self.ref_path[-2, :]
            self.next_waypoint = self.ref_path[-1, :]

        heading_error = self.cal_HE()  # radian
        cross_track_error = self.cal_CTE()  # distance

        # 곡률 계산 및 k 값 조정
        curvature = self.calculate_curvature()
        self.adjust_k_based_on_curvature(curvature)

        # steering 계산 부분 수정
        steering = -heading_error - atan2(self.k * cross_track_error, max(10, self.velocity))  # radian
        
        print(f'heading_error: {heading_error}, cte: {cross_track_error}, steer : {steering}')
        
        # 최대 스티어링 각도를 70도로 제한하고 -1~1 사이로 변환
        max_steering_angle_deg = 70
        steering = steering * 180 / pi
        steering = max(min(steering, max_steering_angle_deg), -max_steering_angle_deg)
        steering = steering / max_steering_angle_deg  # Normalize to -1 to 1
        
        
        print(f'steering: {steering}')
        
        return steering