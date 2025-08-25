#include "imu_spi.h"
#include "MPU9250_REGS.h"

/*IMU센서를 읽기 위한 함수*/
uint8_t imu_spi_read(uint8_t reg_addr)
{
  uint8_t dummy = 0;
  uint8_t data = 0;

  SS_LOW;  //SLAVE선택
  SPI.transfer(0x80 | reg_addr);  //레지스터 읽기모드, 원하는 레지스터 선택
  data = SPI.transfer(dummy); //레지스터의 값을 읽기 위해선 dummy를 한번 보내주면 값을 읽어옴
  SS_HIGH;  //SLAVE해제
  
  return data;
}

/*IMU센서에 명령하기 위한 함수*/
int imu_spi_write(uint8_t reg_addr, uint8_t data)
{
  
  SS_LOW; //SLAVE선택
  SPI.transfer(reg_addr);  //레지스터 쓰기모드, 원하는 레지스터 선택
  SPI.transfer(data);  //레지스터에 값 쓰기
  SS_HIGH;  //SLAVE해제
  
  return 0;
}

/*IMU센서를 여러 값을 읽기 위한 함수*/
int imu_spi_reads(uint8_t reg_addr, uint8_t length, uint8_t *data)  //레지스터, 데이터 길이, 데이터 주소
{
  uint32_t i;
  //uint8_t read_value;

  SS_LOW;//SLAVE선택
  SPI.transfer( reg_addr | 0x80 );  //레지스터 읽기모드, 원하는 레지스터 선택
  for( i=0; i<length; i++ ) //원하는 데이터 길이만큼 값을 저장
  {
    data[i] = SPI.transfer( 0xFF );
  }
  SS_HIGH;//SLAVE해제
  
  return 0;
}

/*IMU센서에 여러 명령을 하기 위한 함수*/
int  imu_spi_writes(uint8_t reg_addr, uint8_t length, uint8_t *data) //레지스터, 데이터 길이, 데이터 주소
{
  uint32_t i;

  SS_LOW;//SLAVE선택
  SPI.transfer( reg_addr ); //레지스터 쓰기모드, 원하는 레지스터 선택

  for( i=0; i<length; i++ )//원하는 데이터 길이만큼 값을 쓰기
  {
    SPI.transfer( data[i] );
  }
  SS_HIGH;//SLAVE해제
  
  delay(1);
  return 0;
}

/*IMU센서의 지자기 센서를 사용하기 위해서 ak8963를 읽기 위한 함수*/
int imu_spi_ak8963_reads(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
  uint8_t index = 0;
  uint8_t status = 0;
  uint32_t timeout = 0;
  uint8_t tmp = 0;

  tmp = akm_addr | MPU9250_I2C_READ;
  imu_spi_writes(MPU9250_I2C_SLV4_ADDR, 1, &tmp);
  delay(1);
  while(index < len){
    tmp = reg_addr + index;
    imu_spi_writes(MPU9250_I2C_SLV4_REG, 1, &tmp);
    delay(1);
    tmp = MPU9250_I2C_SLV4_EN;
    imu_spi_writes(MPU9250_I2C_SLV4_CTRL, 1, &tmp);
    delay(1);

    do {
      if (timeout++ > 50){
        return -2;
      }
      imu_spi_reads(MPU9250_I2C_MST_STATUS, 1, &status);
      delay(2);
    } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
    imu_spi_reads(MPU9250_I2C_SLV4_DI, 1, data + index);
    delay(1);
    index++;
  }
  return 0;
}

/*IMU센서의 지자기 센서를 사용하기 위해서 ak8963에 여러 번 쓰기 위한 함수*/
int imu_spi_ak8963_writes(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
  uint32_t timeout = 0;
  uint8_t status = 0;
  uint8_t tmp = 0;
  uint8_t index = 0;

  tmp = akm_addr;
  imu_spi_writes( MPU9250_I2C_SLV4_ADDR, 1, &tmp);
  delay(2);

  while(index < len){
    tmp = reg_addr + index;
    imu_spi_writes(MPU9250_I2C_SLV4_REG, 1, &tmp);
    delay(2);
    imu_spi_writes( MPU9250_I2C_SLV4_DO, 1, data + index);
    delay(2);
    tmp = MPU9250_I2C_SLV4_EN;
    imu_spi_writes( MPU9250_I2C_SLV4_CTRL, 1, &tmp);
    delay(2);

    do {
      if (timeout++ > 50)
        return -2;
      imu_spi_reads( MPU9250_I2C_MST_STATUS, 1, &status);
      delay(2);
    } while ((status & MPU9250_I2C_SLV4_DONE) == 0);
    if (status & MPU9250_I2C_SLV4_NACK)
      return -3;
    index++;
  }
  return 0;
}

/*IMU센서의 지자기 센서를 사용하기 위해서 ak8963를 쓰기 위한 함수*/
int imu_spi_ak8963_write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data)
{
  uint8_t param[1];

  param[0] = data;

  return imu_spi_ak8963_writes(akm_addr,reg_addr, 1, param);
}
