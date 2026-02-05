#ifndef BMI088_H
#define BMI088_H

#include "stdint.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"




typedef struct {
    // SPI配置
    SpiInitConfig_s spi_acc_config;   // 加速度计SPI配置
    SpiInitConfig_s spi_gyro_config;  // 陀螺仪SPI配置
    
    // 校准配置
    uint8_t enable_calibration;       // 是否启用零偏校准 (1: 启用, 0: 禁用)
} Bmi088InitConfig_s;

typedef struct {
    // SPI实例
    SpiInstance_s* spi_acc;           // 加速度计SPI实例
    SpiInstance_s* spi_gyro;          // 陀螺仪SPI实例
    
    // 回调函数
    void (*spi_module_callback)(SpiInstance_s*); // SPI回调函数
    
    // 传感器数据
    float accel[3];                   // 加速度数据 [x, y, z]
    float gyro[3];                    // 陀螺仪数据 [x, y, z]
    float temperature;                // 温度数据
    
    // 标定参数
    float gyro_offset[3];             // 陀螺仪零偏
    float accel_scale[3];             // 加速度计比例系数
    float g_norm;                     // 重力模长
    float temp_when_cali;             // 校准时温度
    
    // 状态标志
    uint8_t init_status;              // 初始化状态
    uint8_t data_ready;               // 数据就绪标志
    uint8_t cali_offset;              // 是否使用零偏校准
    uint8_t calibration_done;         // 校准完成标志
} Bmi088Instance_s;

// 函数声明
Bmi088Instance_s* Bmi088_Register(Bmi088InitConfig_s *config);
bool Bmi088_Init(Bmi088Instance_s *instance);
//void Bmi088_ReadData(Bmi088Instance_s *instance);

// 寄存器读写函数
bool BMI088_CheckWhoAmI(SpiInstance_s* acc_spi, SpiInstance_s* gyro_spi);
bool BMI088_AccelRead(SpiInstance_s* spi_ins, uint8_t reg, uint8_t* data, uint8_t len);
bool BMI088_GyroRead(SpiInstance_s* spi_ins, uint8_t reg, uint8_t* data, uint8_t len); 
bool BMI088_CheckAccelError(SpiInstance_s* acc_spi);
bool BMI088_ReadAccelData(SpiInstance_s* acc_spi, float* accel_data);
bool BMI088_ReadGyroData(SpiInstance_s* gyro_spi, float* gyro_data) ;
bool BMI088_Read(Bmi088Instance_s *bmi088_ins);
void BMI088_CalibrateOffset(Bmi088Instance_s *bmi088_ins);
uint8_t BMI088_GetCalibrationStatus(Bmi088Instance_s *bmi088_ins);  // 获取校准状态

//温度读取函数
bool BMI088_ReadTemperature(SpiInstance_s* acc_spi, float* temperature);
// 回调函数
void Bmi088_Callback(SpiInstance_s* instance);

#endif // BMI088_H