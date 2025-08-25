#ifndef MCU_H_
#define MCU_H_

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <ctype.h>

// 로봇 물리적 특성 상수
#define WHEEL_RADIUS    0.077f   // 바퀴 반지름 0.077m
#define WHEEL_DISTANCE  0.285f   // 바퀴 중점부터 로봇 중점까지 0.285m

#define PPR             38       // Pulse Per Revolution
#define GEAR_RATIO      104.0f   // 기어비
#define RAD_PER_PULSE   (2 * M_PI / (PPR * GEAR_RATIO))
#define SAMPLING_TIME   0.01f    // 10ms

// 모터 상수
#define Kt              0.036    // 토크 상수
#define R               2        // 저항
#define Ke              0.0305   // 역기전력 상수
#define Jm              1.6e-5   // 관성 모멘트
#define Bm              3.94e-6  // 점성 마찰 계수

// 전기적 특성
#define MAX_VOLTAGE     24.0f    // 모터 최대 Voltage
#define MIN_VOLTAGE     0.0f     // 모터 최소 Voltage

// Motor definitions - 좌우 모터 정의
#define RIGHT           0        // 오른쪽
#define LEFT            1        // 왼쪽
#define WHEEL_NUMBER    2        // 바퀴 수

// PID 제어 상수
#define KP              300.0f   // 비례 게인
#define KI              7.0f     // 적분 게인
#define KAW             (1.0f / 900.0f)  // Anti-windup 게인

// PORT 정의 - LEFT와 RIGHT 명확히 구분
#define DIR_LEFT        PG4      // 왼쪽 모터 방향 핀 (DIR1)
#define DIR_RIGHT       PG3      // 오른쪽 모터 방향 핀 (DIR2)
#define DIR_PORT        PORTG
#define TOP             2000    // PWM 최대값

// 통신 버퍼 크기
#define BUF_SIZE        128

// 모니터링 주기 (ms)
#define MONITOR_PERIOD  100      // 100ms 마다 상태 전송

// 제어 주기(Hz)
#define CONTROL_MOTOR_SPEED_FREQUENCY              30 // 모터 제어주기 -> 초당 30번
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY        30 // jetson에 정보 던지는 주기 -> 초당 30번
#define IMU_PUBLISH_FREQUENCY                      200 // IMU값 받아오는 주기 ->초당 200번

// PI 제어기 구조체
typedef struct {
    float integral;    // 적분값
    float prev_error;  // 이전 오차
    float output;      // 출력값
    float anti_windup; // 안티 와인드업 값
    uint8_t direction; // 방향
} PIController;

// 전역 변수 선언
extern volatile int encoder[WHEEL_NUMBER];        // encoder pulse for controller
extern volatile int encoderReader[WHEEL_NUMBER];  // 엔코더 펄스 수를 세기 위한 변수(조인트 값)

extern volatile float currentSpeed[WHEEL_NUMBER]; // 현재 각속도 (rad/s)
extern volatile float targetSpeed[WHEEL_NUMBER];  // 목표 각속도 (rad/s)

extern PIController pi_controller[WHEEL_NUMBER];  // PI 제어기 구조체

extern char rxBuffer[BUF_SIZE];                   // UART 수신 버퍼
extern volatile int rxIndex;                      // 수신 버퍼 인덱스
extern char txBuffer[BUF_SIZE];                   // UART 송신 버퍼
extern volatile int controlCounter;               // 제어 주기 카운터
extern volatile int monitorCounter;               // 모니터링 주기 카운터

// 자코비안 및 위치 추정 변수
extern float robotX;       // 로봇 X 좌표 (m)
extern float robotY;       // 로봇 Y 좌표 (m)
extern float robotTheta;   // 로봇 방향각 (rad)
extern float d_x;          // X 변화량
extern float d_y;          // Y 변화량
extern float d_theta;      // 각도 변화량

// 함수 선언
void system_init(void);                        // 시스템 초기화
void port_init(void);                          // 포트 초기화
void encoder_init(void);                       // 엔코더 초기화
void timer_init(void);                         // 타이머 초기화
void uart_init(void);                          // UART 초기화

// 모터 제어 함수
void set_motor_direction(uint8_t motor, uint8_t direction);  // 모터 방향 설정
void set_motor_voltage(uint8_t motor, float voltage);        // 모터 전압 설정
void set_motor_speed(float speed_left, float speed_right);   // 모터 속도 설정

// 키네마틱스 및 위치 추정 함수
int parse_kinematics_input(char *input, float *linear_vel, float *angular_vel);  // 키네마틱스 입력 파싱
void inverse_kinematics(float linear_velocity, float angular_velocity, volatile float *speedLeft, volatile float *speedRight);  // 역기구학
void update_robot_position(void);                                                // 로봇 위치 업데이트
void JacobianForCartesianControl(float d_thetaL, float d_thetaR, float W);       // 자코비안 행렬 계산

// 제어 및 통신 함수
float compute_pid(int wheel_index, float target, float current);  // PID 제어 계산
void send_data_to_pc(char *data);                                 // PC로 데이터 전송
void process_command(void);                                        // 명령어 처리
void reset_arr(char *instr, int len);                             // 배열 초기화

// 인터럽트 서비스 루틴
ISR(TIMER2_OVF_vect);     // 타이머2 오버플로우 인터럽트
ISR(INT0_vect);           // 왼쪽 모터 엔코더 인터럽트
ISR(INT2_vect);           // 오른쪽 모터 엔코더 인터럽트
//ISR(USART0_RX_vect);      // UART 수신 인터럽트

#endif // MCU_H
