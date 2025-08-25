#include "AVR.h"
#include "MCU.h"   // MCU 내부 함수들 (set_motor_speed 등) 사용

void myAVR::init() {
	system_init();  // MCU 시스템 초기화
}

void myAVR::controlMotor(const float* goal_velocity_from_cmd) {
	float left = 0.0f;
	float right = 0.0f;

	// 선속도/각속도 → 각 바퀴 속도
	inverse_kinematics(goal_velocity_from_cmd[LINEAR], goal_velocity_from_cmd[ANGULAR], &left, &right);

	set_motor_speed(left, right);  // MCU의 모터 속도 함수 호출
}

void myAVR::receiveCommand(const char* command) {
	float lin = 0.0f;
	float ang = 0.0f;

	if (parse_kinematics_input((char*)command, &lin, &ang)) {
		inverse_kinematics(lin, ang, &targetSpeed[LEFT], &targetSpeed[RIGHT]);
		set_motor_speed(targetSpeed[LEFT], targetSpeed[RIGHT]);
		} else {
		send_data_to_pc("Invalid command format.");  // 에러 메시지 출력
	}
}

void myAVR::readControlGain(char* out, int side) {
	// side는 LEFT 또는 RIGHT지만 현재는 공통 게인 사용
	sprintf(out, "Kp=%.2f Ki=%.2f Kaw=%.5f", KP, KI, KAW);
}

void myAVR::readAngularVelocity(char* out) {
	sprintf(out, "ωL=%.2f rad/s, ωR=%.2f rad/s", currentSpeed[LEFT], currentSpeed[RIGHT]);
}

void myAVR::readEncoderTick(int& left, int& right) {
	left = encoderReader[LEFT];
	right = encoderReader[RIGHT];
}

void myAVR::clearEncoderTick() {
	encoderReader[LEFT] = 0;
	encoderReader[RIGHT] = 0;
}

float myAVR::computeAngularVelocity(int encoder_count) {
	return encoder_count * RADPERPULSE / SAMPLINGTIME;
}
void myAVR::serialWrite(const char* data) {
    send_data_to_pc((char*)data);
}