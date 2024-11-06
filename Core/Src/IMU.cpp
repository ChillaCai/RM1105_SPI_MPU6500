//
// Created by chill on 2024/11/5.
//

#include "IMU.h"
#include "spi.h"
#include <cstdint>

extern uint8_t rx_acc_data[6];
extern uint8_t rx_gyro_data[6];

#define g 9.81

// 片选
void MPU6500_NSS_L(void){
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}
void MPU6500_NSS_H(void){
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
}

void MPU6500_write_byte(uint8_t txdata){
  HAL_SPI_Transmit(&hspi5, &txdata, 1, 1000);
  while (HAL_SPI_GetState(&hspi5) == HAL_SPI_STATE_BUSY_TX);
}

void MPU6500_read_byte(uint8_t *rxdata, uint8_t length){
  HAL_SPI_Receive(&hspi5, rxdata, length, 1000);
  while (HAL_SPI_GetState(&hspi5) == HAL_SPI_STATE_BUSY_RX);
}

void MPU6500_read_reg(uint8_t reg, uint8_t *rxdata, uint8_t length){
  MPU6500_NSS_H();

  MPU6500_write_byte(reg | 0x80);
  MPU6500_read_byte(rxdata, length);

  MPU6500_NSS_L();
}


void IMU::acc_calculate() {
  // 读取量程
  uint8_t rx_acc_range_raw;
  MPU6500_read_reg(0x1C, &rx_acc_range_raw, 1);
  int rx_acc_range = ((int)((rx_acc_range_raw & 0x18) >> 3) + 1) * 2;

  // 读取数据
  MPU6500_read_reg(0x3B, rx_acc_data, 6);

  acc_x_ = (int16_t)((uint16_t)rx_acc_data[1] | (uint16_t) rx_acc_data[0] << 8) / 32768.f * rx_acc_range * g;
  acc_y_ = (int16_t)((uint16_t)rx_acc_data[3] | (uint16_t) rx_acc_data[2] << 8) / 32768.f * rx_acc_range * g;
  acc_z_ = (int16_t)((uint16_t)rx_acc_data[5] | (uint16_t) rx_acc_data[4] << 8) / 32768.f * rx_acc_range * g;
}

void IMU::gyro_calculate() {
  // 读取量程
  uint8_t rx_gyro_range_raw;
  MPU6500_read_reg(0x1B, &rx_gyro_range_raw, 1);
  int rx_gyro_range = ((int)((rx_gyro_range_raw & 0x18) >> 3) + 1) * 250;

  // 读取数据
  MPU6500_read_reg(0x43, rx_gyro_data, 6);

  gyro_x_ = (int16_t)((uint16_t)rx_gyro_data[1] | (uint16_t) rx_gyro_data[0] << 8) / 32768.f * rx_gyro_range ;
  gyro_y_ = (int16_t)((uint16_t)rx_gyro_data[3] | (uint16_t) rx_gyro_data[2] << 8) / 32768.f * rx_gyro_range ;
  gyro_z_ = (int16_t)((uint16_t)rx_gyro_data[5] | (uint16_t) rx_gyro_data[4] << 8) / 32768.f * rx_gyro_range ;
}

