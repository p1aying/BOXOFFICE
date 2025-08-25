#include "MCU.h"

// 전역 변수
volatile int encoder[WHEEL_NUMBER] = {0, 0};        // encoder pulse for controller
volatile int encoderReader[WHEEL_NUMBER] = {0, 0};  // 엔코더 펄스 수를 세기 위한 변수(조인트 값)

volatile float currentSpeed[WHEEL_NUMBER] = {0, 0}; // 현재 각속도 (rad/s)
volatile float targetSpeed[WHEEL_NUMBER] = {0, 0};  // 목표 각속도 (rad/s)

PIController pi_controller[WHEEL_NUMBER] = {0};     // PI 제어기 구조체

char rxBuf[BUF_SIZE] ;
char rxBuffer[BUF_SIZE] = {0};                      // UART 수신 버퍼
volatile int rxIndex = 0;                           // 수신 버퍼 인덱스
char txBuffer[BUF_SIZE] = {0};                      // UART 송신 버퍼
volatile int controlCounter = 0;                    // 제어 주기 카운터
volatile int monitorCounter = 0;                    // 모니터링 주기 카운터

// 자코비안 및 위치 추정 변수
float robotX = 0.0f;             // 로봇 X 좌표 (m)
float robotY = 0.0f;             // 로봇 Y 좌표 (m)
float robotTheta = 0.0f;         // 로봇 방향각 (rad)
float d_x = 0.0f;                // X 변화량
float d_y = 0.0f;                // Y 변화량
float d_theta = 0.0f;            // 각도 변화량

// 함수 선언
void system_init(void);
void port_init(void);
void encoder_init(void);
void timer_init(void);
void uart_init(void);
int parse_kinematics_input(char *input, float *linear_vel, float *angular_vel);
void inverse_kinematics(float linear_velocity, float angular_velocity, volatile float *speedLeft, volatile float *speedRight);
float compute_pid(int wheel_index, float target, float current);
//void apply_motor_pwm(int pwmLeft, int pwmRight);
void send_data_to_pc(char *data);
void update_robot_position(void);
void JacobianForCartesianControl(float d_thetaL, float d_thetaR, float W);
void change_motor_direction(int wheel_index, uint8_t new_direction, float new_speed);
void set_motor_speed(float speed_left, float speed_right);
void process_command(void);
void reset_arr(char *instr, int len);

// 메인 함수
/*
int main(void) {
	
	system_init(); // 시스템 초기화
	sei(); // 인터럽트 활성화
	
	// 메인 루프
	while(1) {
		//apply_motor_pwm(100,0);
	}
	
	
	return 0;
}
*/

// 시스템 초기화 함수
void system_init(void) {
	port_init();
	encoder_init();
	timer_init();
	uart_init();
	
	// 타겟 속도 초기화
	targetSpeed[LEFT] = 0.0f;
	targetSpeed[RIGHT] = 0.0f;
	
	// PI 제어기 초기화
	for (int i = 0; i < WHEEL_NUMBER; i++) {
		pi_controller[i].integral = 0.0f;
		pi_controller[i].prev_error = 0.0f;
		pi_controller[i].output = 0.0f;
		pi_controller[i].anti_windup = 0.0f;
		pi_controller[i].direction = 0;
	}
	
	// 환영 메시지 전송
	send_data_to_pc("Differential Drive Robot Controller Initialized");
}

// 포트 초기화 함수
void port_init(void) {
	// PORTD를 입력으로 설정 (엔코더 입력)
	PORTD |= (1 << PIND1) | (1 << PIND3);  // 엔코더 채널 B 핀에 풀업 적용
	
	// 모터 방향 핀 설정
	DDRG |= (1 << DIR_LEFT) | (1 << DIR_RIGHT);
	
	// PWM 핀 출력 설정 (OC1A=PB5, OC1B=PB6)
	DDRB |= (1 << PB5) | (1 << PB6);
}

// 엔코더 인터럽트 초기화
void encoder_init(void) {
	// 외부 인터럽트 설정 (INT0, INT2)
	EIMSK = (1 << INT2) | (1 << INT0);
	
	// 인터럽트 트리거 방식을 상승 에지로 설정
	EICRA = (1 << ISC21) | (1 << ISC20) | (1 << ISC01) | (1 << ISC00);
}

// 타이머 초기화 함수
void timer_init(void) {
	// Timer1 초기화 (PWM 생성용)
	TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0) | (1 << WGM11) | (0 << WGM10); //PWM초기화
	TCCR1B = (1 << WGM13) | (0 << WGM12) | (0 << CS12) | (1 << CS11) | (0 << CS10);
	ICR1 = TOP;
	
	// Timer2 초기화 (주기적 제어용, 1ms 인터럽트)
	TCCR2 = (0 << WGM21) | (0 << WGM20) | (0 << CS22) | (1 << CS21) | (1 << CS20);  // 분주비 64 설정
	TIMSK |= (1 << TOIE2);  // Timer2 OVF 인터럽트 활성화
	TCNT2 = 0x06;  // 타이머 초기값 설정 ->256-250 = 6
}



// UART 초기화 함수
void uart_init(void) {
	// UART0 초기화 (PC 통신용)
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);  // 수신 인터럽트 활성화, RX/TX 활성화
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 비동기, 패리티 없음, 1 스톱 비트, 8비트 데이터
	UBRR0H = 0;
	UBRR0L = 16;  // 57600 bps @ 16MHz
}

// 역기구학 함수: 선속도와 각속도를 휠 속도로 변환
void inverse_kinematics(float linear_velocity, float angular_velocity, volatile float *speedLeft, volatile float *speedRight) {
	// 단위 변환 (mm/s -> m/s, mrad/s -> rad/s)
	float linear_vel_m = linear_velocity / 1000.0;
	float angular_vel_rad = angular_velocity / 1000.0;
	
	// 두 번째 코드와 일치시킴: 왼쪽/오른쪽 휠 속도 계산
	*speedRight = (linear_vel_m + angular_vel_rad * WHEEL_DISTANCE) / WHEEL_RADIUS;
	*speedLeft = (linear_vel_m - angular_vel_rad * WHEEL_DISTANCE) / WHEEL_RADIUS;
	
	// 디버깅 출력
	//sprintf(txBuffer, "[IK] LinVel=%.3f m/s, AngVel=%.3f rad/s -> Left=%.2f Right=%.2f rad/s",
	//linear_vel_m, angular_vel_rad, *speedLeft, *speedRight);
	//send_data_to_pc(txBuffer);
}

int parse_kinematics_input(char *input, float *linear_vel, float *angular_vel) {
	char l_sign = '+';
	char a_sign = '+';
	int l_value = 0, a_value = 0;

	// 명령 형식 검사 (길이 + 포맷)
	if (strlen(input) < 10|| input[0] != 'L' || input[5] != 'A') {
		reset_arr(input, strlen(input));  // 입력 버퍼 정리
		send_data_to_pc("Fail");          // 오류 알림만 보내고
		return 0;                         // 실패 반환
	}

	// L 부호 및 값 파싱
	l_sign = input[1];
	l_value = (input[2] - '0') * 100 + (input[3] - '0') * 10 + (input[4] - '0');

	// A 부호 및 값 파싱
	a_sign = input[6];
	a_value = (input[7] - '0') * 100 + (input[8] - '0') * 10 + (input[9] - '0');

	// 부호 적용
	*linear_vel = (l_sign == '-') ? -l_value : l_value;  //여기 수정봐야할듯
	*angular_vel = (a_sign == '-') ? -a_value : a_value;

	
	char debug_msg[60];
	sprintf(debug_msg, "[FixedParse] L=%c%d A=%c%d", l_sign, l_value, a_sign, a_value); //목표 각속도 선속도(파싱받은 값)
	send_data_to_pc(debug_msg);

	sprintf(debug_msg, "linear_vel=%.1f, angular_vel=%.1f", *linear_vel, *angular_vel); //목표 각속도 선속도
	send_data_to_pc(debug_msg);

	return 1;  // 성공 반환
}

void reset_arr(char *instr, int len) {
	for (int i = 0; i < len; i++) {
		instr[i] = '\0';
	}
}


// 개선된 PID 제어 함수
float compute_pid(int wheel_index, float target, float current) {
	PIController* pi = &pi_controller[wheel_index];
	float error = target - current;
	float p_term, i_term, output;

	// 비례 항
	p_term = KP * error;

	// 적분 항 + Anti-Windup
	if (fabs(pi->output) < MAX_VOLTAGE) {
		pi->integral += KI * error * SAMPLING_TIME;
		} else {
		pi->integral -= KAW * error;
	}

	// 적분 항 제한
	if (pi->integral > MAX_VOLTAGE) {
		pi->integral = MAX_VOLTAGE;
		} else if (pi->integral < -MAX_VOLTAGE) {
		pi->integral = -MAX_VOLTAGE;
	}

	// 출력 계산
	i_term = pi->integral;
	output = p_term + i_term;

	// 출력 제한 (양/음수 모두 허용)
	if (output > MAX_VOLTAGE) {
		output = MAX_VOLTAGE;
		} else if (output < -MAX_VOLTAGE) {
		output = -MAX_VOLTAGE;
	}

	// 출력 변화율 제한 (스무딩)
	float max_change_rate = 0.1f;
	if (fabs(output - pi->output) > max_change_rate) {
		if (output > pi->output) {
			output = pi->output + max_change_rate;
			} else {
			output = pi->output - max_change_rate;
		}
	}

	pi->output = output;
	return output;
}



void set_motor_direction(uint8_t motor, uint8_t direction) {
	if (motor == 0) {
		if (direction == 0) DIR_PORT |= (1 << DIR_LEFT);  // A 모터 정방향
		else DIR_PORT &= ~(1 << DIR_LEFT);                // A 모터 역방향
		} else {
		if (direction == 0) DIR_PORT &= ~(1 << DIR_RIGHT); // B 모터 정방향 (반대방향)
		else DIR_PORT |= (1 << DIR_RIGHT);                 // B 모터 역방향 (반대방향)
	}
}


// 모터 PWM 적용 함수 - 수정됨: OC1A=LEFT, OC1B=RIGHT
void set_motor_voltage(uint8_t motor, float voltage) {
	// 방향 추출
	uint8_t direction = (voltage >= 0) ? 0 : 1;
	float abs_voltage = fabs(voltage);

	// PWM 계산
	uint16_t pwm = (uint16_t)(abs_voltage * 2000.0 / MAX_VOLTAGE);
	if (pwm > 2000) pwm = 2000;

	// 모터별 설정
	if (motor == 0) {
		set_motor_direction(0, direction);  // ✅ 부호 기반 DIR 설정
		OCR1A = pwm;
		} else {
		set_motor_direction(1, direction);
		OCR1B = pwm;
	}
}



// 모터 속도 설정 함수

void set_motor_speed(float speed_left, float speed_right) {
	// 모터 왼쪽(LEFT) 설정
	if (speed_left >= 0) {
		pi_controller[LEFT].direction = 0;  // 정방향
		} else {
		pi_controller[LEFT].direction = 1;  // 역방향
	}
	
	// 모터 오른쪽(RIGHT) 설정
	if (speed_right >= 0) {
		pi_controller[RIGHT].direction = 0;  // 정방향
		} else {
		pi_controller[RIGHT].direction = 1;  // 역방향
	}
	
	// 두 모터 모두 속도가 0일 때 PI 제어기 초기화
	if (speed_left == 0 && speed_right == 0) 
	{
		targetSpeed[LEFT] = 0;
		targetSpeed[RIGHT] = 0;
		
		// PI 제어기 상태 초기화
		for (int i = 0; i < WHEEL_NUMBER; i++) {
			pi_controller[i].integral = 0;
			pi_controller[i].prev_error = 0;
			pi_controller[i].output = 0;
			pi_controller[i].anti_windup = 0;
		}
	}
	else 
	{
		// 정상 속도 설정
		targetSpeed[LEFT] = speed_left;
		targetSpeed[RIGHT] = speed_right;
	}
	
	// 디버깅 출력
	//sprintf(txBuffer, "[MOTOR] LeftSpd=%.2f Dir=%d, RightSpd=%.2f Dir=%d",
	//targetSpeed[LEFT], pi_controller[LEFT].direction,
	//targetSpeed[RIGHT], pi_controller[RIGHT].direction);
	//send_data_to_pc(txBuffer);
}
// PC로 데이터 전송 함수
void send_data_to_pc(char *data) {
	int i = 0;
	
	while (data[i] != '\0') {
		while (!(UCSR0A & (1 << UDRE0)));  // 송신 버퍼가 비어있을 때까지 대기
		UDR0 = data[i++];
	}
	
	// 줄바꿈 문자 전송
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = '\r';
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = '\n';
}

// 로봇 위치 업데이트 함수
void update_robot_position(void) {
	// 휠 회전량을 통해 로봇 위치 추정
	float d_thetaL = currentSpeed[LEFT] * SAMPLING_TIME;
	float d_thetaR = currentSpeed[RIGHT] * SAMPLING_TIME;
	
	// 자코비안 행렬을 이용한 위치 변화량 계산
	JacobianForCartesianControl(d_thetaL, d_thetaR, robotTheta);
	
	// 로봇 위치 업데이트
	robotX += d_x;
	robotY += d_y;
	robotTheta += d_theta;
	
	// 각도를 -π ~ π 범위로 정규화
	while (robotTheta > M_PI) robotTheta -= 2 * M_PI;
	while (robotTheta < -M_PI) robotTheta += 2 * M_PI;
}

// 자코비안 행렬을 이용한 직교 좌표계 변환 함수
void JacobianForCartesianControl(float d_thetaL, float d_thetaR, float W) {
	d_x = (WHEEL_RADIUS/2) * cos(W) * (d_thetaL + d_thetaR);
	d_y = (WHEEL_RADIUS/2) * sin(W) * (d_thetaL + d_thetaR);
	d_theta = (WHEEL_RADIUS/WHEEL_DISTANCE) * (d_thetaR - d_thetaL);
}

// 명령어 처리 함수
void process_command(void) {
	switch(rxBuffer[0]) {
		case 'L': {
			float linear_vel = 0.0f;
			float angular_vel = 0.0f;

			// 파싱 성공시에만 역기구학 & 속도 설정 수행
			if (parse_kinematics_input(rxBuffer, &linear_vel, &angular_vel)) {
				inverse_kinematics(linear_vel, angular_vel, &targetSpeed[LEFT], &targetSpeed[RIGHT]);
				set_motor_speed(targetSpeed[LEFT], targetSpeed[RIGHT]);

				sprintf(txBuffer, "Set speeds: L=%.1f A=%.1f, Left=%.2f Right=%.2f rad/s",
				linear_vel/1000.0f, angular_vel/1000.0f,
				targetSpeed[LEFT], targetSpeed[RIGHT]);
				send_data_to_pc(txBuffer);
				} else {
				// 오류시 이전 속도 유지 (모터 멈추지 않음)
				send_data_to_pc("Ignored invalid command, keeping previous speed");
			}
			break;
	}
		default:
		send_data_to_pc("Unknown command");
		break;
	}
}

// 타이머2 오버플로우 인터럽트 (1ms마다 발생)
ISR(TIMER2_OVF_vect) {
	controlCounter++;
	monitorCounter++;
	
	if (controlCounter >= 10) {  // 10ms마다 속도 제어 수행
		// 현재 속도 계산
		currentSpeed[LEFT] = encoder[LEFT] * RAD_PER_PULSE / SAMPLING_TIME;
		currentSpeed[RIGHT] = encoder[RIGHT] * RAD_PER_PULSE / SAMPLING_TIME;
		
		//sprintf(txBuffer, "[SPD] ωL=%.2f ωR=%.2f", currentSpeed[LEFT], currentSpeed[RIGHT]);
		//send_data_to_pc(txBuffer);
		
		// 로봇 위치 업데이트
		//update_robot_position();
		
		//sprintf(txBuffer, "[SPD] targetL=%.2f targetR=%.2f", targetSpeed[LEFT], targetSpeed[RIGHT]);
		//send_data_to_pc(txBuffer);
		// PID 제어 계산
		float outputLeft = compute_pid(LEFT, targetSpeed[LEFT], currentSpeed[LEFT]);
		float outputRight = compute_pid(RIGHT, targetSpeed[RIGHT], currentSpeed[RIGHT]);
		
		// PWM 값으로 변환 (24V를 10000으로 스케일링)
		set_motor_voltage(1, outputRight);
		set_motor_voltage(0, outputLeft);
		
		
		// 모터 제어
		// 각속도 읽어오기 엔코더 읽어오기
		//sprintf(txBuffer, "[T2 ISR] ωL=%.2f ωR=%.2f ENC L=%d R=%d",
		//currentSpeed[LEFT], currentSpeed[RIGHT],
		//encoder[LEFT], encoder[RIGHT]);
		//send_data_to_pc(txBuffer);
		
		// 엔코더 카운트 초기화
		encoder[LEFT] = 0;
		encoder[RIGHT] = 0;
		
		// 카운터 초기화
		controlCounter = 0;
	}
	
	// 타이머 초기값 재설정
	TCNT2 = 6;
}

// 인터럽트 핸들러 - 왼쪽 모터 엔코더 (INT0)
ISR(INT0_vect) {
	if (EICRA & (1 << ISC00)) {  // 상승 에지에서 발생한 인터럽트
		if (PIND & (1 << PIND1)) {  // 채널 A가 HIGH
			encoder[LEFT]--;
			encoderReader[LEFT]--;
			} else {  // 채널 A가 LOW
			encoder[LEFT]++;
			encoderReader[LEFT]++;
		}
		} else {  // 하강 에지에서 발생한 인터럽트
		if (PIND & (1 << PIND1)) {  // 채널 A가 HIGH
			encoder[LEFT]++;
			encoderReader[LEFT]++;
			} else {  // 채널 A가 LOW
			encoder[LEFT]--;
			encoderReader[LEFT]--;
		}
	}
	
	// 다음 인터럽트를 위해 에지 방향 전환
	EICRA ^= (1 << ISC00); // 상승에지/하강에지 모두 받을 수 있게 구성
}

// 인터럽트 핸들러 - 오른쪽 모터 엔코더 (INT2)
ISR(INT2_vect) {
	if (EICRA & (1 << ISC20)) {  // 상승 에지에서 발생한 인터럽트
		if (PIND & (1 << PIND3)) {  // 채널 B가 HIGH
			encoder[RIGHT]--;
			encoderReader[RIGHT]--;
			} else {  // 채널 A가 LOW
			encoder[RIGHT]++;
			encoderReader[RIGHT]++;
		}
		} else {  // 하강 에지에서 발생한 인터럽트
		if (PIND & (1 << PIND3)) {  // 채널 B가 HIGH
			encoder[RIGHT]++;
			encoderReader[RIGHT]++;
			} else {  // 채널 A가 LOW
			encoder[RIGHT]--;
			encoderReader[RIGHT]--;
		}
	}
	
	// 다음 인터럽트를 위해 에지 방향 전환
	EICRA ^= (1 << ISC20);
}

// UART 수신 인터럽트 핸들러
// UART 수신 인터럽트 핸들러
/*
ISR(USART0_RX_vect) {
	char received = UDR0;
	
	// 개행 문자를 받으면 명령어 처리
	if (received == '\n' || received == '\r') {
		if (rxIndex > 0) {
			rxBuffer[rxIndex] = '\0';  // 문자열 종료
			process_command();         // 명령 처리
			rxIndex = 0;               // 버퍼 인덱스 초기화
			memset(rxBuffer, 0, BUF_SIZE);
		}
	}
	// 일반 문자인 경우 버퍼에 저장
	else if (rxIndex < BUF_SIZE - 1) {
		rxBuffer[rxIndex++] = received;
	}
	// 버퍼 오버플로우 방지
	else {
		rxIndex = 0;
		memset(rxBuffer, 0, BUF_SIZE);
		send_data_to_pc("Buffer overflow, command ignored");
	}
}
*/
