#!/usr/bin/env python3
import sys
import select
import tty
import termios
import rospy
from geometry_msgs.msg import Twist

# 속도 상수
MAX_LIN_VEL = 0.45  # m/s로 변경 (원래 450을 1000으로 나눠서 0.45)
MAX_ANG_VEL = 0.45  # rad/s로 변경
LIN_VEL_STEP_SIZE = 0.01  # m/s 단위로 변경
ANG_VEL_STEP_SIZE = 0.01  # rad/s 단위로 변경

# 방향 기억용
last_linear_dir = 1
last_angular_dir = 1

# 이전 속도 기억용 (변경 감지용)
prev_linear_vel = 0.0
prev_angular_vel = 0.0

msg = """
Control Your Robot with Keyboard!
---------------------------
↑/↓ : 직진 속도 증가/감소
←/→ : 회전 속도 증가/감소
Space bar : 정지
CTRL-C : 종료
"""

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            first = sys.stdin.read(1)
            if first == '\x1b':
                second = sys.stdin.read(1)
                third = sys.stdin.read(1)
                return first + second + third
            else:
                return first
        else:
            return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def constrain(val, min_val, max_val):
    return max(min(val, max_val), min_val)

def publish_twist(linear_vel, angular_vel):
    global prev_linear_vel, prev_angular_vel
    
    try:
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        cmd_vel_pub.publish(twist)
        
        # 속도가 변경되었을 때만 출력
        if linear_vel != prev_linear_vel or angular_vel != prev_angular_vel:
            rospy.loginfo(f"Publishing: Linear={linear_vel:.3f} m/s, Angular={angular_vel:.3f} rad/s")
            prev_linear_vel = linear_vel
            prev_angular_vel = angular_vel
            
    except Exception as e:
        rospy.logerr(f"ROS publish error: {str(e)}")

def main():
    global cmd_vel_pub, last_linear_dir, last_angular_dir
    
    rospy.init_node('teleop_keyboard')
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    
    print(msg)
    
    try:
        while not rospy.is_shutdown():
            key = get_key()
            
            if key == '\x1b[A':  # ↑
                if target_linear_velocity == 0 and last_linear_dir < 0:
                    target_linear_velocity = LIN_VEL_STEP_SIZE
                else:
                    target_linear_velocity = constrain(
                        target_linear_velocity + LIN_VEL_STEP_SIZE, 
                        -MAX_LIN_VEL, MAX_LIN_VEL
                    )
                if target_linear_velocity > 0:
                    last_linear_dir = 1
                elif target_linear_velocity < 0:
                    last_linear_dir = -1
                    
            elif key == '\x1b[B':  # ↓
                if target_linear_velocity == 0 and last_linear_dir > 0:
                    target_linear_velocity = -LIN_VEL_STEP_SIZE
                else:
                    target_linear_velocity = constrain(
                        target_linear_velocity - LIN_VEL_STEP_SIZE, 
                        -MAX_LIN_VEL, MAX_LIN_VEL
                    )
                if target_linear_velocity > 0:
                    last_linear_dir = 1
                elif target_linear_velocity < 0:
                    last_linear_dir = -1
                    
            elif key == '\x1b[C':  # →
                target_angular_velocity = constrain(
                    target_angular_velocity - ANG_VEL_STEP_SIZE, 
                    -MAX_ANG_VEL, MAX_ANG_VEL
                )
                if target_angular_velocity > 0:
                    last_angular_dir = 1
                elif target_angular_velocity < 0:
                    last_angular_dir = -1
                    
            elif key == '\x1b[D':  # ←
                target_angular_velocity = constrain(
                    target_angular_velocity + ANG_VEL_STEP_SIZE, 
                    -MAX_ANG_VEL, MAX_ANG_VEL
                )
                if target_angular_velocity > 0:
                    last_angular_dir = 1
                elif target_angular_velocity < 0:
                    last_angular_dir = -1
                    
            elif key == ' ':  # Space bar
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                
            elif key == '\x03':  # Ctrl+C
                break
            
            # cmd_vel 토픽 발행
            publish_twist(target_linear_velocity, target_angular_velocity)
            
            rate.sleep()
            
    except Exception as e:
        rospy.logerr(f"에러 발생: {e}")
        
    finally:
        # 종료 시 정지 명령 발행
        publish_twist(0.0, 0.0)
        rospy.loginfo("Teleop keyboard 종료")

if __name__ == '__main__':
    main()