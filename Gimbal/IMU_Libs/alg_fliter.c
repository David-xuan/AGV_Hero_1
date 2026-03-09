/**
 * @file alg_fliter.c
 * @brief 这是一个用于集成各种有用的滤波器，算法或者各种纯数学代码的库
 * @author CGH 
 * @date 2025-7-6
 */

#include "alg_fliter.h"
#include <stdio.h>
#include "arm_math.h"
#include <math.h>

float d;
float a0;
float y;
float a1;
float a2;
float a;
//写在前面：这个库虽然看起来是能够实例化滤波器，实际上一个滤波器只能对应一个调用者，懒得写动态内存分配了（很不优雅！）（不是）

////基于arm的浮点数取余函数
//float32_t arm_fmod(float32_t x, float32_t y) {
//    float32_t quotient;
//    arm_divide_f32(x, y, &quotient);  // 计算 x/y 的商（浮点除法）
//    quotient = truncf(quotient);    // 向零取整
//    return x - quotient * y;        // 余数 = x - (商 * y)
//}


// 最速控制综合函数 fhan()
float fhan(float x1, float x2, float r, float h0) {
     d = r * h0 * h0;
     a0 = h0 * x2;
     y = x1 + a0;
     a1 = sqrtf(d * (d + 8 * fabsf(y)));
     a2 = a0 + (y > 0 ? 1 : -1) * (a1 - d) / 2;
     a = 0;

    if (fabsf(y) > d) {
        a = a2;
    } else {
        a = a0 + y / h0;
    }

    float fhan_output = 0;
    if (fabsf(a) > d) {
        fhan_output = -r * (a > 0 ? 1 : -1);
    } else {
        fhan_output = -r * a / d;
    }

    return fhan_output;
}



/**
 * @brief ADRC最速跟踪微分器 (TD) 的核心函数
 * @param x1 状态误差 (x1_k - target)
 * @param x2 状态微分 (x2_k)
 * @param r  速度因子 (决定跟踪快慢)
 * @param h  步长/采样周期
 * @return   最优的控制加速度
 */
float fhan_correct(float x1, float x2, float r, float h) {
    float d, d0, y, a0, a;

    d = r * h;
    d0 = h * d;
    y = x1 + h * x2;
    a0 = sqrtf(d * d + 8.0f * r * fabsf(y));

    if (fabsf(y) > d0) {
        a = x2 + (a0 - d) / 2.0f * ((y > 0) ? 1.0f : -1.0f);
    } else {
        a = x2 + y / h;
    }

    if (fabsf(a) > d) {
        return -r * ((a > 0) ? 1.0f : -1.0f);
    } else {
        return -r * a / d;
    }
}

/**
 * @brief 滑动平均滤波器(我觉得很优雅的版本)
 * @param input 输入数据指针
 * @param output 输出数据指针
 * @param fliter_cfg 滤波器配置
 */
void Move_aver_fliter(float32_t *input, float32_t *output, fliter_config *fliter_cfg) {
    // 使用编译时常量和位操作
    enum { 
        BUFFER_BITS = 3,                    // log2(8) = 3
        BUFFER_SIZE = 1 << BUFFER_BITS,     // 2^3 = 8
        BUFFER_MASK = BUFFER_SIZE - 1       // 0b111 = 7
    };
    
    static float32_t buffer[BUFFER_SIZE] = {0};
    static uint32_t index = 0;              // 使用32位，避免频繁的类型转换
    static uint32_t count = 0;
    static float32_t running_sum = 0.0f;    // 维护运行总和
    
    // 限制滤波器大小并确保为2的幂
    uint8_t filter_size = fliter_cfg->blockSize;
    filter_size = (filter_size > BUFFER_SIZE) ? BUFFER_SIZE : filter_size;
    
    // 如果buffer已满，减去即将被覆盖的值
    if (count >= filter_size) {
        uint8_t old_index = (index - filter_size) & BUFFER_MASK;
        running_sum -= buffer[old_index];
    } else {
        count++;
    }
    
    // 添加新值
    buffer[index & BUFFER_MASK] = input[0];
    running_sum += input[0];
    
    // 索引递增（利用整数溢出实现自然环形）
    //这个是因为，无论index是多少，只要index每增大8，其二进制数据中低3位的数据永远按照0123456的顺序递增
    //因此，index的低3位总是循环使用buffer的索引，也就实现了高效的自然索引循环。
    //实际上这个可以等价于index对于缓冲区长度取模（整除取余）。
    index++;
    
    // 计算平均值
    output[0] = running_sum / count;
}

/**
 * @brief 低通滤波器
 * @param input 输入数据指针
 * @param output 输出数据指针
 * @param fliter_cfg 滤波器配置
 */
void Lowpass_fliter(float32_t *input, float32_t *output, fliter_config *fliter_cfg){
    static float32_t prev_output = 0.0f; // 上一个输出值
    static uint8_t initialized = 0;      // 初始化标志
    
    float32_t alpha = 0.1f; // 默认低通滤波器系数
    
    // 计算滤波器系数
    // freq[0] = 采样频率 , freq[1] = 截止频率 （单位都是Hz）
    if (fliter_cfg->freq[0] > 0 && fliter_cfg->freq[1] > 0) {
        // 正确的公式：α = dt/(τ + dt)
        // 其中：dt = 1/fs (采样周期), τ = 1/(2π*fc) (时间常数)
        float32_t dt = 1.0f / fliter_cfg->freq[0];           // 采样周期
        float32_t tau = 1.0f / (2.0f * PI * fliter_cfg->freq[1]); // 时间常数
        alpha = dt / (tau + dt);
        
        // 限制alpha范围，避免数值问题
        alpha= (alpha > 1.0f)? 1.0f:alpha ;
        alpha= (alpha < 0.0f)? 0.0f:alpha ;
    }
    
    // 初始化处理
    if (!initialized) {
        prev_output = input[0];  // 用第一个输入值初始化
        initialized = 1;
        output[0] = input[0];
        return;
    }
    
    // 一阶RC低通滤波：y[n] = α*x[n] + (1-α)*y[n-1]
    output[0] = alpha * input[0] + (1.0f - alpha) * prev_output;
    
    // 更新上一个输出值
    prev_output = output[0];
}









// ============== 新的实例化滤波器实现 ==============

/**
 * @brief 注册滑动平均滤波器实例
 * @param config 滤波器初始化配置
 * @return 滤波器实例指针，失败返回NULL
 */
MovingAvgFilter_t* MovingAvgFilter_Register(FilterInitConfig_t *config) {
    if (config == NULL) {
        return NULL;
    }
    
    // 动态分配内存
    MovingAvgFilter_t *filter = (MovingAvgFilter_t *)pvPortMalloc(sizeof(MovingAvgFilter_t));
    if (filter == NULL) {
        return NULL;
    }
    
    // 初始化滤波器状态
    memset(filter->buffer, 0, sizeof(filter->buffer));
    filter->index = 0;
    filter->count = 0;
    filter->running_sum = 0.0f;
    filter->filter_size = (config->filter_size > 8) ? 8 : config->filter_size;
    filter->initialized = 1;
    
    return filter;
}

/**
 * @brief 注册低通滤波器实例
 * @param config 滤波器初始化配置
 * @return 滤波器实例指针，失败返回NULL
 */
LowpassFilter_t* LowpassFilter_Register(FilterInitConfig_t *config) {
    if (config == NULL || config->sample_freq <= 0 || config->cutoff_freq <= 0) {
        return NULL;
    }
    
    // 动态分配内存
    LowpassFilter_t *filter = (LowpassFilter_t *)pvPortMalloc(sizeof(LowpassFilter_t));
    if (filter == NULL) {
        return NULL;
    }
    
    // 初始化滤波器状态
    filter->prev_output = 0.0f;
    filter->initialized = 0;
    filter->cutoff_freq = config->cutoff_freq;
    filter->sample_freq = config->sample_freq;
    
    // 计算滤波系数
    float32_t dt = 1.0f / config->sample_freq;
    float32_t tau = 1.0f / (2.0f * PI * config->cutoff_freq);
    filter->alpha = dt / (tau + dt);
    
    // 限制alpha范围
    if (filter->alpha > 1.0f) filter->alpha = 1.0f;
    if (filter->alpha < 0.0f) filter->alpha = 0.0f;
    
    return filter;
}

/**
 * @brief 滑动平均滤波器处理函数
 * @param filter 滤波器实例
 * @param input 输入数据
 * @param output 输出数据指针
 */
void MovingAvgFilter_Process(MovingAvgFilter_t *filter, float32_t input, float32_t *output) {
    if (filter == NULL || output == NULL) {
        return;
    }
    
    const uint8_t BUFFER_MASK = 7; // 8-1 = 7, 用于位运算取模
    
    // 如果缓冲区已满，减去要被覆盖的旧值
    if (filter->count >= filter->filter_size) {
        uint8_t old_index = (filter->index - filter->filter_size) & BUFFER_MASK;
        filter->running_sum -= filter->buffer[old_index];
    } else {
        filter->count++;
    }
    
    // 添加新值
    filter->buffer[filter->index & BUFFER_MASK] = input;
    filter->running_sum += input;
    
    // 更新索引
    filter->index++;
    
    // 计算平均值
    *output = filter->running_sum / filter->count;
}

/**
 * @brief 低通滤波器处理函数
 * @param filter 滤波器实例
 * @param input 输入数据
 * @param output 输出数据指针
 */
void LowpassFilter_Process(LowpassFilter_t *filter, float32_t input, float32_t *output) {
    if (filter == NULL || output == NULL) {
        return;
    }
    
    // 初始化处理
    if (!filter->initialized) {
        filter->prev_output = input;
        filter->initialized = 1;
        *output = input;
        return;
    }
    
    // 一阶RC低通滤波：y[n] = α*x[n] + (1-α)*y[n-1]
    *output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
    filter->prev_output = *output;
}

/**
 * @brief 释放滑动平均滤波器实例
 * @param filter 滤波器实例指针
 */
void MovingAvgFilter_Free(MovingAvgFilter_t *filter) {
    if (filter != NULL) {
        vPortFree(filter);
    }
}

/**
 * @brief 释放低通滤波器实例
 * @param filter 滤波器实例指针
 */
void LowpassFilter_Free(LowpassFilter_t *filter) {
    if (filter != NULL) {
        vPortFree(filter);
    }
}



/**
 * @brief 快速平方根倒数计算函数
 * @param x 输入值
 * @return 输入值的平方根倒数
 */
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
/**
 * @brief 符号函数
 * @param x 输入值
 * @return 符号函数结果
 */
int sgn(int x){
    return x == 0 ? 0 : x > 0 ? 1 : -1;
}
/**
 * @brief 类符号函数
 * @param x 输入值
 * @param d 死区范围
 * @return 类符号函数结果
 */
int fsgn(float x) {
    return (x != 0.0f ? (x < 0.0f ? -1 : 1) : 0);
}
/**
 * @brief 带死区的类符号函数
 * @param x 输入值
 * @param d 死区范围
 * @return 带死区的类符号函数结果
 */
float sgn_like(float x, float d) {
    if (fabs(x) >= d)
        return fsgn(x);
    else
        return x / d;
}