
/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @author  annotation and modification by NeoZeng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention INS任务的初始化不要放入实时系统!应该由application拥有实例,随后在
 *            应用层调用初始化函数.
 *
 ******************************************************************************
 */
#ifndef __APP_INS_TASK_H
#define __APP_INS_TASK_H

#include "stdint.h"
#include "dev_ist_8310.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
//#include "alg_quaternionEKF.h"  //已经接入，也可以切换成mahony
#include "MadgwickAHRS.h"
#include "Fusion_AHRS.h"
#include "alg_pid.h"  //温度控制相关

#include "math.h"
#include "dev_bmi088.h"          // BMI088传感器
#include "alg_fliter.h"          // 滤波器
#include "bsp_pwm.h"             // PWM控制
#include "tim.h"                 // 定时器

//CMSIS好像要求用float32_t，可能是为了可移植性考虑。我这里就不管了



typedef struct
{
  float32_t A_x;
  float32_t A_y;
  float32_t A_z;
}acceleration;
typedef struct
{
    // 改为欧拉角输出（角度制）
    float32_t roll;    // 横滚角（度）
      float32_t pitch;   // 俯仰角（度）
    float32_t yaw;     // 偏航角（度）
    acceleration Acc;  // 保留加速度数据
    float Gyro[3]; // 三轴陀螺仪数据
    uint8_t ins_ready; // INS数据就绪标志
}quaternions_struct_t;


void isttask(void const * argument);

// INS任务相关函数声明


uint8_t Quater_Init(float* origin_quater,uint8_t check);
uint8_t caculate_angle(const float* gyro, const float* origin_quater, float* quaternion, float dt);
uint8_t calculate_quaternion_from_gravity( float32_t* g0,  float32_t* g1, float32_t* quaternion);
void quaternion_multiply(const float* q1, const float* q2, float* result);
void quaternion_to_euler(const float* quaternion, float* roll, float* pitch, float* yaw);
void quaternion_normalize(float* quaternion);
void quaternion_update(float* origin_quater);
float calculate_norm(const float *arr);

// // EKF辅助函数
// void get_ekf_euler_angles(float* roll, float* pitch, float* yaw);
// uint8_t get_ekf_status(void);

/**
 * @brief EKF使用示例
 * @note 
 *   // 获取EKF四元数
 *   float quaternion[4];
 *   quaternion_update(quaternion);
 *   
 *   // 获取EKF欧拉角
 *   float roll, pitch, yaw;
 *   get_ekf_euler_angles(&roll, &pitch, &yaw);
 *   
 *   // 检查EKF收敛状态
 *   if (get_ekf_status()) {
 *       // EKF已收敛，姿态数据可靠
 *   }
 *   
 *   // 直接访问EKF结构体
 *   float temperature_compensated_yaw = QEKF_INS.YawTotalAngle; // 包含圈数的连续偏航角
 *   float gyro_bias_x = QEKF_INS.GyroBias[0]; // 陀螺仪X轴零偏估计
 */

// 全局变量声明
extern quaternions_struct_t Quater;           // 四元数结构体
extern PidInstance_s *ins_pid;                // INS PID控制器
extern uint8_t test_data[5];                  // 磁力计测试数据

// EKF相关外部访问接口
//extern QEKF_INS_t QEKF_INS;                   // EKF状态结构体（来自alg_quaternionEKF.c）


#endif
// /**
//  ******************************************************************************
//  * @file    ins_task.h
//  * @author  Wang Hongxi
//  * @author  annotation and modification by NeoZeng
//  * @version V2.0.0
//  * @date    2022/2/23
//  * @brief
//  ******************************************************************************
//  * @attention INS任务的初始化不要放入实时系统!应该由application拥有实例,随后在
//  *            应用层调用初始化函数.
//  *
//  ******************************************************************************
//  */
// #ifndef __APP_INS_TASK_H
// #define __APP_INS_TASK_H

// #include "stdint.h"
// #include "dev_ist_8310.h"
// #include "FreeRTOS.h"
// #include "task.h"
// #include "main.h"
// #include "cmsis_os.h"
// //#include "alg_quaternionEKF.h"  //已经接入，也可以切换成mahony

// void isttask(void const * argument);



// #endif
