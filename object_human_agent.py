#!/usr/bin/env python

import rospy
import tf
import math
from tf.transformations import euler_from_quaternion
from derived_object_msgs.msg import ObjectArray, Object
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Imu

class CarlaListener:
    def __init__(self):
        self.ego_yaw = None
        self.ego_position = Point()  # Point 객체로 초기화
        self.lead_vehicle_status_pub = rospy.Publisher('/carla/lead_vehicle_status', Twist, queue_size=10)

    def quaternion_to_yaw(self, quaternion):
        # 쿼터니언을 사용하여 Yaw(heading) 각도를 계산
        euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        yaw = euler[2]  # roll = euler[0], pitch = euler[1], yaw = euler[2]
        return yaw

    def calculate_distance(self, pos1, pos2):
        # 두 점 사이의 유클리드 거리 계산
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

    def imu_callback(self, data):
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        )
        euler_angles = euler_from_quaternion(quaternion)
        self.ego_yaw = euler_angles[2]

    def odometry_callback(self, msg):
        self.ego_position.x = msg.pose.pose.position.x
        self.ego_position.y = msg.pose.pose.position.y

    def angle_mod(self, angle):
        # 각도를 -pi에서 pi 사이로 변환
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def object_callback(self, data):
        # /carla/objects 토픽에서 받은 데이터를 처리하는 콜백 함수
        closest_object = None
        min_distance = float('inf')
        candidate_ids = []

        if self.ego_yaw is not None and self.ego_position is not None:
            for obj in data.objects:
                # 내 차량과 같은 위치인지 확인하여 제외
                if self.calculate_distance(self.ego_position, obj.pose.position) < 2:
                    continue  # 내 차량은 건너뜀

                # classification이 6인지 확인
                if obj.classification == 6:
                    obj_yaw = self.quaternion_to_yaw(obj.pose.orientation)
                    
                    # 헤딩이 같은 방향으로 향하는 객체 필터링 (예: ±10도 내외)
                    if abs(self.angle_mod(obj_yaw - self.ego_yaw)) < 0.174533:  # 10도 = 0.174533 라디안
                        # Y 기준으로 ego vehicle과 ±1.75m 이내의 객체 필터링
                        if abs(obj.pose.position.y - self.ego_position.y) <= 1.75:
                            distance = self.calculate_distance(self.ego_position, obj.pose.position)
                            candidate_ids.append(obj.id)  # 후보군의 ID를 리스트에 추가
                            if distance < min_distance:
                                min_distance = distance
                                closest_object = obj
        
        if closest_object is not None:
            # 후보군과 선택된 리드 차량의 ID를 출력
            rospy.loginfo(f"Lead Vehicle 후보군 IDs: {candidate_ids}")
            rospy.loginfo(f"선택된 Lead Vehicle ID: {closest_object.id}")

            # 위치와 헤딩 및 속도 계산
            position_x = closest_object.pose.position.x
            position_y = closest_object.pose.position.y

            # 속도는 x, y의 선속도를 기반으로 계산
            velocity = math.sqrt(closest_object.twist.linear.x**2 + closest_object.twist.linear.y**2)

            # 헤딩 계산
            heading_yaw = self.quaternion_to_yaw(closest_object.pose.orientation)

            # Twist 메시지를 사용하여 위치, 속도, 헤딩 퍼블리시
            status_msg = Twist()

            # linear.x와 linear.y에 위치 정보를 저장
            status_msg.linear.x = position_x
            status_msg.linear.y = position_y

            # linear.z에 속도 정보를 저장
            status_msg.linear.z = velocity

            # angular.z에 헤딩 정보를 저장
            status_msg.angular.z = heading_yaw

            rospy.loginfo(f"Lead Vehicle Position: x={position_x}, y={position_y}")
            rospy.loginfo(f"Lead Vehicle Speed: {velocity} m/s")
            rospy.loginfo(f"Lead Vehicle Heading: {heading_yaw} radians")
            rospy.loginfo("----------------------------")

            self.lead_vehicle_status_pub.publish(status_msg)

        else:
            # 조건을 만족하는 객체가 없을 경우
            rospy.loginfo("Lead vehicle 후보군이 없습니다.")
            status_msg = Twist()

            # 모든 필드를 0으로 설정
            status_msg.linear.x = 0
            status_msg.linear.y = 0
            status_msg.linear.z = 0
            status_msg.angular.z = 0

            self.lead_vehicle_status_pub.publish(status_msg)

    def listener(self):
        # ROS 노드 초기화
        rospy.init_node('carla_listener', anonymous=True)
        
        # /carla/ego_vehicle/odometry 토픽 구독
        rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.odometry_callback)
        rospy.Subscriber("/carla/ego_vehicle/imu", Imu, self.imu_callback)
        # /carla/objects 토픽 구독
        rospy.Subscriber("/carla/objects", ObjectArray, self.object_callback)
        
        # ROS가 종료될 때까지 계속 대기
        rospy.spin()

if __name__ == '__main__':
    listener = CarlaListener()
    listener.listener()
