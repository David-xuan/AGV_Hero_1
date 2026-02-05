#ifndef ALG_FLITER_H
#define ALG_FLITER_H
#include "arm_math.h" // ARM CMSIS数学库
#include "FreeRTOS.h"

//定义回调函数类型
typedef struct {
    float32_t *pData;      // 数据指针
    uint32_t blockSize;    // 块大小(每次处理的数据大小)
    arm_fir_instance_f32 *S; // FIR状态结构体
    float32_t freq[2]; // 采样频率
    uint8_t type ; // 滤波器类型
} fliter_config;

// 滑动平均滤波器实例结构体
typedef struct {
    float32_t buffer[8];        // 环形缓冲区
    uint32_t index;             // 当前索引
    uint32_t count;             // 有效数据个数
    float32_t running_sum;      // 运行总和
    uint8_t filter_size;        // 滤波器大小
    uint8_t initialized;        // 初始化标志
} MovingAvgFilter_t;

// 低通滤波器实例结构体
typedef struct {
    float32_t prev_output;      // 上一次输出
    float32_t alpha;            // 滤波系数
    uint8_t initialized;        // 初始化标志
    float32_t cutoff_freq;      // 截止频率
    float32_t sample_freq;      // 采样频率
} LowpassFilter_t;

// 滤波器初始化配置
typedef struct {
    uint8_t filter_size;        // 滑动平均窗口大小
    float32_t cutoff_freq;      // 低通滤波截止频率
    float32_t sample_freq;      // 采样频率
} FilterInitConfig_t;

typedef void (*FilterCallback)(float32_t input, fliter_config *config, float32_t *output);

// 原有滤波器函数（保留）
void Move_aver_fliter(float32_t *input, float32_t *output, fliter_config *fliter_cfg) ;
void Lowpass_fliter(float32_t *input, float32_t *output, fliter_config *fliter_cfg);

// 新的实例化滤波器函数
MovingAvgFilter_t* MovingAvgFilter_Register(FilterInitConfig_t *config);
LowpassFilter_t* LowpassFilter_Register(FilterInitConfig_t *config);
void MovingAvgFilter_Process(MovingAvgFilter_t *filter, float32_t input, float32_t *output);
void LowpassFilter_Process(LowpassFilter_t *filter, float32_t input, float32_t *output);
void MovingAvgFilter_Free(MovingAvgFilter_t *filter);
void LowpassFilter_Free(LowpassFilter_t *filter);
float32_t fhan(float32_t x1, float32_t x2, float32_t r, float32_t h0);
float fhan_correct(float x1, float x2, float r, float h);
#endif