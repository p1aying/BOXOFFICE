#ifndef SENSOR_H_
#define SENSOR_H_

#include <ros.h>
#include <IMU.h>
#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/MagneticField.h>
//#include <geometry_msgs/Quaternion.h>

#define ACCEL_FACTOR                      0.000598550415   // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g]
                                                           //                                             Scale : +- 16384
#define GYRO_FACTOR                       0.0010642        // (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s]
                                                           //                                             Scale : +- 16.4[deg/s]
#define MAG_FACTOR                        15e-8

class Sensor
{
 public:
  bool init(void);

  // IMU
  void initIMU(void);
  sensor_msgs::Imu getIMU(void);
  void updateIMU(void);
  void calibrationGyro(void);

  float * getOrientation(void);
  //sensor_msgs::MagneticField getMag(void);

 private:
  sensor_msgs::Imu           imu_msg_;
  //sensor_msgs::MagneticField mag_msg_;

  cIMU imu_;
};

#endif // SENSOR_H_