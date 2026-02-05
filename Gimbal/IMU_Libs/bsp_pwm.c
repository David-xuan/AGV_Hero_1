/**
* @file bsp_pwm.c
 * @author zhan xile
 * @brief 对pwm进行封库
 * @version 0.1
 * @date 2025-07-2
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "string.h"
#include "stdint.h"
#include "bsp_pwm.h"
#include "FreeRTOS.h"
static uint8_t idx = 0;                                      // 当前PWM实例索引
static PwmInstance_s* pwm_instance[PWM_DEVICE_CNT] = {NULL}; // PWM实例指针数组

/**
 * @brief 获取定时器时钟频率F
 * @param htim TIM句柄指针
 * @return uint32_t 时钟频率(Hz)
 */
static uint32_t Pwm_SelectTclk(TIM_HandleTypeDef* htim) {
    uint32_t pclk;
    uint32_t apb_div;

    // 获取系统时钟频率
    RCC_ClkInitTypeDef clk_init;
    uint32_t flash_latency;
    HAL_RCC_GetClockConfig(&clk_init, &flash_latency);

    // 判断定时器在哪个APB总线上
    if (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM9 ||
        htim->Instance == TIM10 || htim->Instance == TIM11) {
        // APB2总线上的定时器
        pclk = HAL_RCC_GetPCLK2Freq();
        apb_div = clk_init.APB2CLKDivider;
        }
    else {
        // APB1总线上的定时器
        pclk = HAL_RCC_GetPCLK1Freq();
        apb_div = clk_init.APB1CLKDivider;
    }

    // 如果APB分频系数不为1，定时器时钟需要乘以2
    if (apb_div != RCC_HCLK_DIV1) {
        pclk *= 2;
    }

    return pclk;
}

PwmInstance_s* Pwm_Register(PwmInitConfig_s* config) {
    // 检查实例数量是否超过限制
    if (idx >= PWM_DEVICE_CNT) {
        return NULL; // 返回NULL而不是死循环，更友好
    }

    // 分配内存并初始化
    PwmInstance_s* pwm = (PwmInstance_s*)pvPortMalloc(sizeof(PwmInstance_s));
    if (pwm == NULL) {
        vPortFree(pwm);
        return NULL; // 内存分配失败
    }
    memset(pwm, 0, sizeof(PwmInstance_s));

    // 填充实例参数
    pwm->htim = config->htim;
    pwm->channel = config->channel;
    pwm->period = config->period;
    pwm->duty_ratio = config->duty_ratio;
    pwm->callback = config->callback;
    pwm->id = config->id;
    pwm->tclk = Pwm_SelectTclk(pwm->htim);

    // 配置PWM参数
    Pwm_SetPeriod(pwm, pwm->period);
    Pwm_SetDutyRatio(pwm, pwm->duty_ratio);

    // 启动PWM
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);

    // 保存实例指针
    pwm_instance[idx++] = pwm;

    return pwm;
}

bool Pwm_Start(PwmInstance_s* pwm) {
    if (pwm != NULL) {
        HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
        return true; // 启动成功
    }
    return false;
}

bool Pwm_Stop(PwmInstance_s* pwm) {
    if (pwm != NULL) {
        HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
        return true;
    }
    return false;
}

bool Pwm_SetPeriod(PwmInstance_s* pwm, float period) {
    if (pwm == NULL || pwm->htim == NULL) return false;

    // 计算ARR值: ARR = (周期 * 时钟频率) / (预分频 + 1)
    uint32_t arr = (uint32_t)(period * (pwm->tclk / (pwm->htim->Init.Prescaler + 1)));
    __HAL_TIM_SetAutoreload(pwm->htim, arr);
    pwm->period = period;

    // 保持占空比不变，更新比较值
    Pwm_SetDutyRatio(pwm, pwm->duty_ratio);
    return true;
}

bool Pwm_SetDutyRatio(PwmInstance_s* pwm, float duty_ratio) {
    if (pwm == NULL || pwm->htim == NULL) return false;

    // 限制占空比范围
    if (duty_ratio < 0.0f) duty_ratio = 0.0f;
    if (duty_ratio > 1.0f) duty_ratio = 1.0f;

    // 计算比较值: CCR = 占空比 * ARR
    uint32_t ccr = (uint32_t)(duty_ratio * __HAL_TIM_GetAutoreload(pwm->htim));
    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, ccr);
    pwm->duty_ratio = duty_ratio;
    return true;
}

bool Pwm_StartDMA(PwmInstance_s* pwm, uint32_t* pData, uint32_t Size) {
    if (pwm != NULL && pwm->htim != NULL) {
        HAL_TIM_PWM_Start_DMA(pwm->htim, pwm->channel, pData, Size);
        return true; // 启动DMA传输成功
    }
    return false;
}

bool Pwm_Unregister(PwmInstance_s* pwm) {
    if (pwm == NULL) return false;

    // 停止PWM
    Pwm_Stop(pwm);

    // 从实例数组中移除
    for (uint8_t i = 0; i < idx; i++) {
        if (pwm_instance[i] == pwm) {
            // 将后面的实例前移
            for (uint8_t j = i; j < idx - 1; j++) {
                pwm_instance[j] = pwm_instance[j + 1];
            }
            pwm_instance[idx - 1] = NULL;
            idx--;
            break;
        }
    }

    // 释放FreeRTOS分配的内存
    vPortFree(pwm);
    return true;
}
