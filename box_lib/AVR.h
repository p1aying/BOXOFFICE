#ifndef AVR_H_
#define AVR_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <Arduino.h>
#include "MCU.h"

#define BUF_SIZE 128
#define F_CPU            160000000UL
#define WHEEL_NUMBER     2
#define LEFT             0
#define RIGHT            1
#define LINEAR           0
#define ANGULAR          1
#define SAMPLINGTIME     0.01f
#define RADPERPULSE      1.59e-3                  // encoder�� 1���� pulse �� ȸ���� (���� : rad/pulse)

#define CONTROL_MOTOR_SPEED_FREQUENCY              30
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY        30
#define IMU_PUBLISH_FREQUENCY                      200



class myAVR {
	public:

	void init();                                            // 시스템 초기화 (MCU 초기화 함수 호출)
	void controlMotor(const float* goal_velocity_from_cmd); // 제어 명령 입력 (선속도, 각속도)
	void receiveCommand(const char* command);               // 문자열 명령 직접 파싱 (ex. "L+100A+050")
	void readControlGain(char* out, int side);              // PID 게인 값 읽기
	void readAngularVelocity(char* out);                    // 각속도 읽기 (문자열 출력)
	void readEncoderTick(int& left, int& right);            // 엔코더 값 읽기
	void clearEncoderTick();                                // 엔코더 초기화
	void serialWrite(const char* data);
	private:
	float computeAngularVelocity(int encoder_count);// (옵션) 필요한 내부 연산 함수
};


#endif /* AVR_H_ */
