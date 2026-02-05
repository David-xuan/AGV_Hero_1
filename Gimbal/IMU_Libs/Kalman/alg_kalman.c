#include "arm_math.h"
#include "alg_kalman.h"//规范命名
#include "math.h"

//新建协方差矩阵,矩阵中的每个元素 Σij 表示第 i 个和第 j 个状态变量之间的相关度

static float32_t kalman_gain[2][2]; //协方差矩阵

//预测矩阵（暂定为2x2矩阵）
//预测矩阵是一个描述系统状态转移的矩阵，它将当前状态映射到下一个状态
static float32_t kalman_predict[2][2]; //预测矩阵
//测量矩阵
//测量矩阵是一个将系统状态映射到测量空间的矩阵，它将系统状态转换为可观测的输出
static float32_t kalman_measure[2][2]; //测量矩阵
//测量矩阵和预测矩阵的高斯乘积就是最佳估计
//状态向量