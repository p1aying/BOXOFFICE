#ifndef _IMU_SPI_H_
#define _IMU_SPI_H_

#include <avr/io.h>
#include <SPI.h>
#include <stdint.h>

#define SS_HIGH (PORTB) |= ((1)<<(PORTB0))  //HIGH일때 SLAVE해제
#define SS_LOW (PORTB) &= (~((1)<<(PORTB0)))    //HIGH일때 SLAVE선택

int imu_spi_reads(uint8_t reg_addr, uint8_t length, uint8_t *data); /*IMU센서를 여러 값을 읽기 위한 함수*/
int  imu_spi_writes(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);   /*IMU센서에 여러 명령을 하기 위한 함수*/
int imu_spi_write(uint8_t reg_addr, uint8_t data);  /*IMU센서에 명령하기 위한 함수*/
uint8_t imu_spi_read(uint8_t reg_addr); /*IMU센서를 읽기 위한 함수*/

int imu_spi_ak8963_reads(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data);   /*IMU센서의 지자기 센서를 사용하기 위해서 ak8963를 읽기 위한 함수*/
int imu_spi_ak8963_writes(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data);  /*IMU센서의 지자기 센서를 사용하기 위해서 ak8963에 여러 번 쓰기 위한 함수*/
int imu_spi_ak8963_write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data); /*IMU센서의 지자기 센서를 사용하기 위해서 ak8963를 쓰기 위한 함수*/

#endif
