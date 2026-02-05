/**
 * @file bsp_pwm.h
 * @author zhan xile
 * @brief 对pwm进行封库
 * @version 0.1
 * @date 2025-07-2
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef _BSP_PWM_H_
#define _BSP_PWM_H_

#include "tim.h"
#include "stdint.h"
#include "stdbool.h"
#define PWM_DEVICE_CNT 16 // 支持的最大PWM设备数量

/* pwm实例结构体 */
typedef struct pwm_ins_temp {
    TIM_HandleTypeDef* htim;                // TIM句柄
    uint32_t channel;                       // 通道
    uint32_t tclk;                          // 时钟频率
    float period;                           // 周期
    float duty_ratio;                       // 占空比
    void (*callback)(struct pwm_ins_temp*); // DMA传输完成回调函数
    void* id;                               // 实例ID
} PwmInstance_s;

typedef struct {
    TIM_HandleTypeDef* htim;          // TIM句柄
    uint32_t channel;                 // 通道
    float period;                     // 周期
    float duty_ratio;                 // 占空比
    void (*callback)(PwmInstance_s*); // DMA传输完成回调函数
    void* id;                         // 实例ID
} PwmInitConfig_s;


/**
 * @brief 注册一个pwm实例
 *
 * @param config 初始化配置
 * @return PWMInstance*
 */
PwmInstance_s* Pwm_Register(PwmInitConfig_s* config);

bool Pwm_Start(PwmInstance_s* pwm);                                    //启动pwm
bool Pwm_SetDutyRatio(PwmInstance_s* pwm, float duty_ratio);           //设置pwm占空比
bool Pwm_Stop(PwmInstance_s* pwm);                                     //停止pwm
bool Pwm_SetPeriod(PwmInstance_s* pwm, float period);                  //设置pwm周期
bool Pwm_StartDMA(PwmInstance_s* pwm, uint32_t* pData, uint32_t Size); //启动pwm dma传输
bool Pwm_Unregister(PwmInstance_s* pwm);                               //注销pwm实例

#endif
