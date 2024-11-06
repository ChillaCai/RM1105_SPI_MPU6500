//
// Created by chill on 2024/11/5.
//

#ifndef RM11_IMU_A_IMU_H
#define RM11_IMU_A_IMU_H

#include <cstdint>

class IMU {
private:
  float acc_x_;
  float acc_y_;
  float acc_z_;
  float gyro_x_;
  float gyro_y_;
  float gyro_z_;
public:
  void acc_calculate();
  void gyro_calculate();
};

void MPU6500_Init();

void MPU6500_gyro_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length);
void BMI088_accel_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length);

#endif // RM11_IMU_A_IMU_H
