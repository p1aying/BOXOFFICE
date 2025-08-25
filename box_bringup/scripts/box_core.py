#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import time
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class BoxbotCore:
    def __init__(self):
        rospy.init_node('boxbot_core', anonymous=True)
        
        # ROS 퍼블리셔/서브스크라이버
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        # tf 브로드캐스터 사용
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        # 시리얼 통신 설정
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 57600)
        self.ser = None
        
        # 오도메트리 데이터
        self.info = [0.0, 0.0, 0.0, 0.0, 0.0]  # x, y, theta, vx, vth
        self.packet = [0, 0, 0, 0, 0, 0]
        
        # 필터링을 위한 변수
        self.last_valid_info = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.odom_buffer = []
        self.buffer_size = 5
        
        # 급격한 변화 감지를 위한 임계값
        self.MAX_DELTA_POS = 0.3  # 최대 위치 변화 (m)
        self.MAX_DELTA_ANGLE = 0.3  # 최대 각도 변화 (rad)
        self.MAX_VELOCITY = 2.0  # 최대 속도 (m/s)
        self.MAX_ANGULAR_VEL = 3.0  # 최대 각속도 (rad/s)
        
        # 명령어 관련
        self.cmd_array = "L+000A+000\n"  # 초기 정지 명령
        self.last_linear_dir = 1
        self.last_angular_dir = 1
        
        # Rate 설정
        self.pub_rate = rospy.Rate(30)
        self.write_rate = rospy.Rate(20)
        
        # 시리얼 연결
        self.connect_serial()
        
        # 스레드 설정
        self.write_thread = None
        self.read_thread = None
    
    def connect_serial(self):
        """향상된 시리얼 연결"""
        while not rospy.is_shutdown():
            try:
                self.ser = serial.Serial(
                    self.port, 
                    self.baud, 
                    timeout=3,
                    write_timeout=3,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False
                )
                
                # 버퍼 크기 설정
                try:
                    self.ser.set_buffer_size(rx_size=4096, tx_size=4096)
                except:
                    pass  # 일부 시스템에서 지원하지 않을 수 있음
                
                rospy.loginfo("Serial connected: %s" % self.port)
                break
            except serial.SerialException:
                rospy.logwarn("Serial connection failed, retrying...")
                time.sleep(1)
                continue
    
    def cmd_vel_callback(self, msg):
        """cmd_vel 콜백"""
        linear_vel = int(msg.linear.x * 1000)
        angular_vel = int(msg.angular.z * 1000)
        
        # 방향 저장
        if linear_vel > 0:
            self.last_linear_dir = 1
        elif linear_vel < 0:
            self.last_linear_dir = -1
            
        if angular_vel > 0:
            self.last_angular_dir = 1
        elif angular_vel < 0:
            self.last_angular_dir = -1
        
        # 프로토콜 형식
        if linear_vel != 0:
            linear_str = ("+" if linear_vel > 0 else "-") + "%03d" % abs(linear_vel)
        else:
            linear_str = "+000" if self.last_linear_dir >= 0 else "-000"
            
        if angular_vel != 0:
            angular_str = ("+" if angular_vel > 0 else "-") + "%03d" % abs(angular_vel)
        else:
            angular_str = "+000" if self.last_angular_dir >= 0 else "-000"
            
        self.cmd_array = "L%sA%s\n" % (linear_str, angular_str)
    
    def write(self):
        """write 스레드"""
        while not rospy.is_shutdown():
            try:
                if self.ser and self.ser.is_open:
                    self.ser.write(self.cmd_array.encode())
                    self.ser.flush()
                self.write_rate.sleep()
            except Exception as e:
                rospy.logerr("Write error: %s" % e)
                self.handle_serial_error()
    
    def validate_odom_data(self, new_info):
        """오도메트리 데이터 유효성 검사"""
        # 급격한 위치 변화 감지
        delta_x = abs(new_info[0] - self.last_valid_info[0])
        delta_y = abs(new_info[1] - self.last_valid_info[1])
        delta_th = abs(new_info[2] - self.last_valid_info[2])
        
        # 속도 제한
        if abs(new_info[3]) > self.MAX_VELOCITY or abs(new_info[4]) > self.MAX_ANGULAR_VEL:
            rospy.logwarn("Velocity exceeds limit: vx=%.3f, vth=%.3f" % (new_info[3], new_info[4]))
            return False
        
        # 급격한 변화 감지
        if delta_x > self.MAX_DELTA_POS or delta_y > self.MAX_DELTA_POS or delta_th > self.MAX_DELTA_ANGLE:
            rospy.logwarn("Large jump detected: dx=%.3f, dy=%.3f, dth=%.3f" % (delta_x, delta_y, delta_th))
            return False
        
        return True
    
    def filter_odom(self, new_info):
        """이동 평균 필터"""
        self.odom_buffer.append(new_info[:])
        
        if len(self.odom_buffer) > self.buffer_size:
            self.odom_buffer.pop(0)
        
        # 평균 계산
        avg_info = [0.0] * 5
        for data in self.odom_buffer:
            for i in range(5):
                avg_info[i] += data[i]
        
        for i in range(5):
            avg_info[i] /= len(self.odom_buffer)
        
        return avg_info
    
    def read(self):
        """read 스레드 - 필터링 추가"""
        while not rospy.is_shutdown():
            try:
                if self.ser and self.ser.is_open and self.ser.readable():
                    # 버퍼 비우기
                    if self.ser.in_waiting > 100:  # 너무 많은 데이터가 쌓이면
                        self.ser.reset_input_buffer()
                    
                    arr = self.ser.readline()
                    
                    if arr and arr[0] == 'a':
                        # 파싱
                        for i in range(6):
                            self.packet[i] = arr.find(chr(97+i))
                        
                        new_info = [0.0] * 5
                        for i in range(5):
                            if self.packet[i] != -1 and self.packet[i+1] != -1:
                                try:
                                    new_info[i] = float(arr[self.packet[i]+1:self.packet[i+1]])
                                except ValueError:
                                    rospy.logwarn("Parse error at index %d" % i)
                                    new_info = self.last_valid_info[:]
                                    break
                        
                        # 데이터 유효성 검사
                        if self.validate_odom_data(new_info):
                            # 필터링 적용
                            filtered_info = self.filter_odom(new_info)
                            self.info = filtered_info
                            self.last_valid_info = filtered_info[:]
                            
                            rospy.loginfo("Odom: x=%.3f, y=%.3f, th=%.3f, vx=%.3f, vth=%.3f" % 
                                        (self.info[0], self.info[1], self.info[2], self.info[3], self.info[4]))
                        else:
                            # 유효하지 않은 데이터는 이전 값 유지
                            self.info = self.last_valid_info[:]
                            
            except Exception as e:
                rospy.logerr("Read error: %s" % e)
                self.handle_serial_error()
    
    def handle_serial_error(self):
        """시리얼 에러 처리"""
        # 정지 명령 전송
        self.cmd_array = "L+000A+000\n"
        
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(self.cmd_array.encode())
                self.ser.close()
        except:
            pass
        
        # 재연결
        self.connect_serial()
    
    def run(self):
        """메인 실행 루프"""
        self.write_thread = threading.Thread(target=self.write)
        self.write_thread.daemon = True
        self.read_thread = threading.Thread(target=self.read)
        self.read_thread.daemon = True
        
        self.write_thread.start()
        self.read_thread.start()
        
        while not rospy.is_shutdown():
            try:
                current_time = rospy.Time.now()
                
                # tf.transformations 사용
                odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.info[2])
                
                # TF 브로드캐스트
                self.odom_broadcaster.sendTransform(
                    (self.info[0], self.info[1], 0.),
                    odom_quat,
                    current_time,
                    "base_footprint",
                    "odom"
                )
                
                # 오도메트리 발행
                odom = Odometry()
                odom.header.stamp = current_time
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_footprint"
                odom.pose.pose = Pose(Point(self.info[0], self.info[1], 0.), 
                                     Quaternion(*odom_quat))
                odom.twist.twist = Twist(Vector3(self.info[3], 0, 0), 
                                       Vector3(0, 0, self.info[4]))
                
                # 공분산 추가 (선택사항)
                odom.pose.covariance[0] = 0.001  # x
                odom.pose.covariance[7] = 0.001  # y
                odom.pose.covariance[35] = 0.001  # theta
                
                self.odom_pub.publish(odom)
                self.pub_rate.sleep()
                
            except Exception as e:
                rospy.logerr("Run error: %s" % e)
                self.handle_serial_error()


if __name__ == '__main__':
    try:
        robot = BoxbotCore()
        robot.run()
    except rospy.ROSInterruptException:
        pass