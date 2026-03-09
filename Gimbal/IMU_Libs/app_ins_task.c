/**
 * @file app_ins_task.c
 * @brief INS任务的实现文件 - 使用EKF进行高精度姿态解算
 * @author CGH and HWX
 * @date 2025-07-07
 * @note 
 *   - 已启用EKF（扩展卡尔曼滤波）进行姿态融合
 *   - 使用双级滤波（滑动平均+低通）处理IMU数据
 *   - 集成温度控制系统，目标温度40°C
 *   - 1000Hz实时姿态解算，提供四元数和欧拉角输出
 */
//本来想在ins task和底层IMU之间加个抽象层的，跃鹿没加我也不加了
//本任务基本上都使用arm_math库，因此都采用float32_t
#include "app_ins_task.h"
#include "FreeRTOS.h"
#include "bsp_dwt.h"  // 添加DWT头文件
#include "Fusion_AHRS.h"
#include "MadgwickAHRS.h"

//extern Ist8310Instance_s *asdf;
MadgwickAHRS *madgwick_ahrs;  
FusionAhrs fusion_ahrs;
FusionOffset fusion_offset;
PidInstance_s *ins_pid;
uint8_t test_data[5]={0,0,0,0,0};
quaternions_struct_t Quater;//四元数
Bmi088Instance_s *bmi088_test;
float32_t Gyro_Offset[3] = {0};        // 陀螺仪零偏
  Bmi088InitConfig_s bmi088_config = {
    .spi_acc_config = {
        .spi_handle = &hspi1,                    // SPI句柄
        .GPIOx = GPIOA,                          // 加速度计CS引脚端口
        .cs_pin = GPIO_PIN_4,                    // 加速度计CS引脚
        .spi_work_mode = SPI_BLOCKING_MODE,            // 阻塞模式
        .spi_module_callback = Bmi088_Callback,  // SPI回调函数
        .id = NULL,                              // 父模块指针（将在注册时设置）
        .tx_len = 8,                             // 发送缓冲区长度
        .rx_len = 8,                             // 接收缓冲区长度
    },
    .spi_gyro_config = {
        .spi_handle = &hspi1,                    // SPI句柄
        .GPIOx = GPIOB,                          // 陀螺仪CS引脚端口
        .cs_pin = GPIO_PIN_0,                    // 陀螺仪CS引脚
        .spi_work_mode = SPI_BLOCKING_MODE,            // 中断模式
        .spi_module_callback = Bmi088_Callback,  // SPI回调函数
        .id = NULL,                              // 父模块指针（将在注册时设置）
        .tx_len = 8,                             // 发送缓冲区长度
        .rx_len = 8,                             // 接收缓冲区长度
    },
    
    // 校准配置
    .enable_calibration = 1,                     // 启用零偏校准（1: 启用, 0: 禁用）
  };

// 温度控制PID配置
PidInitConfig_s temp_pid_config = {
    .kp = 15.0f,        // 比例系数
    .ki = 0.0f,         // 积分系数  
    .kd = 0.0f,           // 微分系数
    .kf = 0.0f,           // 前馈系数
    .angle_max = 0.0f,    // 不需要角度限幅
    .i_max = 1000.0f,     // 积分限幅
    .out_max = 20.0f,     // 输出限幅(占空比0-100%,这里用0-20对应0-100%)
    .dead_zone = 0.0f,    // 死区
    .i_variable_min = 0.0f, // 变速积分下限
    .i_variable_max = 0.0f, // 变速积分上限
    .d_first = 0          // 微分先行关闭
};

// PWM配置 - TIM10 CH1
PwmInitConfig_s pwm_config = {
    .htim = &htim10,           // 使用TIM10
    .channel = TIM_CHANNEL_1,   // 通道1
    .period = 0.02f,           // 20ms周期 (50Hz)
    .duty_ratio = 0.0f,        // 初始占空比0%
    .callback = NULL,          // 不需要回调
    .id = NULL                 // 不需要ID
};

FusionAhrsSettings settings = {
    .convention = FusionConventionNwu,  // 坐标系：NWU（北西上）
    .gain = 0.5f,                      // 算法增益
    .gyroscopeRange = 0.0f,            // 禁用量程检测重置（避免34.9rad/s满量程时误重置）
    .accelerationRejection = 7.0f,    // 加速度计拒绝阈值（度）
    .recoveryTriggerPeriod = 5000,        // 恢复触发周期
};







//测试代码
float test=0;

//测试代码结束




static float sensor_offset_r_body[3] = {0.0f, 0.0f, 0.0f}; // IMU 到旋转中心的偏移（m）
static float last_gyro_filtered[3] = {0.0f, 0.0f, 0.0f};
static float last_alpha_filtered[3] = {0.0f, 0.0f, 0.0f};
static const float alpha_lpf_tau = 0.02f; // alpha LPF 时间常数 (s)
static const float omega_thresh = 0.5f;   // 当 |ω| < 阈值时不补偿 (rad/s)

/**
 * @brief Transform 3dvector from BodyFrame to EarthFrame
 * @param[1] vector in BodyFrame
 * @param[2] vector in EarthFrame
 * @param[3] quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief Transform 3dvector from EarthFrame to BodyFrame
 * @param[1] vector in EarthFrame
 * @param[2] vector in BodyFrame
 * @param[3] quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief Compensate centrifugal and Euler acceleration from accelerometer
 * @param[in] gyro_filtered Filtered gyro [3] (rad/s)
 * @param[in] accel_filtered Filtered accel [3] (m/s^2)
 * @param[in] dt Time step (s)
 * @param[out] compensated_accel Compensated accel [3] (m/s^2)
 */
void compensate_centrifugal_accel(const float *gyro_filtered, const float *accel_filtered, float dt, float *compensated_accel)
{
    // 计算角加速度 alpha（rad/s^2），并低通
    float alpha[3];
    for (int i = 0; i < 3; ++i) {
        alpha[i] = (gyro_filtered[i] - last_gyro_filtered[i]) / dt;
        float alpha_lpf = last_alpha_filtered[i] + (dt / (alpha_lpf_tau + dt)) * (alpha[i] - last_alpha_filtered[i]);
        last_alpha_filtered[i] = alpha_lpf;
        alpha[i] = alpha_lpf;
    }
    // 更新 last_gyro_filtered
    for (int i = 0; i < 3; ++i) {
        last_gyro_filtered[i] = gyro_filtered[i];
    }

    // 计算离心和欧拉加速度（body frame）
    float tmp[3] = {
        gyro_filtered[1]*sensor_offset_r_body[2] - gyro_filtered[2]*sensor_offset_r_body[1],
        gyro_filtered[2]*sensor_offset_r_body[0] - gyro_filtered[0]*sensor_offset_r_body[2],
        gyro_filtered[0]*sensor_offset_r_body[1] - gyro_filtered[1]*sensor_offset_r_body[0]
    };
    float a_cent[3] = {
        gyro_filtered[1]*tmp[2] - gyro_filtered[2]*tmp[1],
        gyro_filtered[2]*tmp[0] - gyro_filtered[0]*tmp[2],
        gyro_filtered[0]*tmp[1] - gyro_filtered[1]*tmp[0]
    };
    float a_euler[3] = {
        alpha[1]*sensor_offset_r_body[2] - alpha[2]*sensor_offset_r_body[1],
        alpha[2]*sensor_offset_r_body[0] - alpha[0]*sensor_offset_r_body[2],
        alpha[0]*sensor_offset_r_body[1] - alpha[1]*sensor_offset_r_body[0]
    };
    float a_compensate[3] = { a_cent[0] + a_euler[0], a_cent[1] + a_euler[1], a_cent[2] + a_euler[2] };

    // 选择性应用补偿
    float omega_norm = sqrtf(gyro_filtered[0]*gyro_filtered[0] + gyro_filtered[1]*gyro_filtered[1] + gyro_filtered[2]*gyro_filtered[2]);
    if (omega_norm > omega_thresh) {
        for (int i = 0; i < 3; ++i) {
            compensated_accel[i] = accel_filtered[i] - a_compensate[i];
        }
    } else {
        for (int i = 0; i < 3; ++i) {
            compensated_accel[i] = accel_filtered[i];
        }
    }
}




/**
 * @brief 1khz运行的INS任务
 * @details 该任务用于处理惯性导航系统（INS）的数据，执行卡尔曼滤波和姿态估计等操作。
 *        p.s. osDelay(1);
 *
 */
 
void isttask(void const * argument)
{
  /* USER CODE BEGIN isttask */
    
    // 声明外部BMI088实例和遥控器实例
    
    bmi088_test = Bmi088_Register(&bmi088_config);  
  
  // 检查注册是否成功
  if (bmi088_test == NULL) {
    // BMI088注册失败，可以添加错误处理
    Error_Handler();
  }
    
    
    
    // 注册温度控制PID和PWM实例
    PidInstance_s *temp_pid = Pid_Register(&temp_pid_config);
    PwmInstance_s *heater_pwm = Pwm_Register(&pwm_config);
    
    // 【新增】在上电初期强制等待温度达到 40 度
    // 只有在温度稳定后，陀螺仪零偏才不会剧烈漂移，AHRS 的高增益初始化才有意义
    float init_temp = 0.0f;
    while (init_temp < 39.5f) {
        if (BMI088_ReadTemperature(bmi088_test->spi_acc, &init_temp)) {
            // 在等待过程中持续运行 PID 控温逻辑
            float pid_output = Pid_Calculate(temp_pid, 40.0f, init_temp);
            float duty_ratio = pid_output / 20.0f;
            if (duty_ratio < 0.0f) duty_ratio = 0.0f;
            if (duty_ratio > 1.0f) duty_ratio = 1.0f;
            Pwm_SetDutyRatio(heater_pwm, duty_ratio);
        }
        osDelay(2); // 2ms 周期
    }

    // 实例化滤波器配置
    FilterInitConfig_t filter_config = {
        .filter_size = 5,              // 5点滑动平均
        .cutoff_freq = 50.0f,          // 50Hz截止频率
        .sample_freq = 1000.0f         // 1000Hz采样频率
    };
    
    // 只为加速度计和陀螺仪的XYZ轴创建滑动平均滤波器实例
    MovingAvgFilter_t *accel_moving_filters[3];    // 加速度计滑动平均滤波器
    MovingAvgFilter_t *gyro_moving_filters[3];     // 陀螺仪滑动平均滤波器

    // 注册滑动平均滤波器实例
    for (int i = 0; i < 3; i++) {
        accel_moving_filters[i] = MovingAvgFilter_Register(&filter_config);
        gyro_moving_filters[i] = MovingAvgFilter_Register(&filter_config);
        // 检查内存分配是否成功
        if (accel_moving_filters[i] == NULL || gyro_moving_filters[i] == NULL) {
            Error_Handler();
        }
    }
    
    // 姿态解算相关变量
    float32_t filtered_accel[3] = {0};      // 滤波后的加速度数据
    float32_t filtered_gyro[3] = {0};       // 滤波后的陀螺仪数据
    
    float temperature = 0.0f;               // 温度数据
    float32_t current_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 当前四元数
    float32_t origin_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // 初始四元数
    uint8_t ins_initialized = 0;           // INS初始化标志
    
    
    // 初始化四元数（简化版，实际应用中需要更复杂的初始化）
	madgwick_ahrs=pvPortMalloc(sizeof(MadgwickAHRS));
    Quater_Init(origin_quaternion, 1); // 使用默认初始化
  
    ins_initialized = 1;
    
    
    
    // DWT计时变量
    uint32_t dwt_cnt_last = 0;
    float dt = 0.001f;  // 初始dt
    static float lasttime = 0;
    uint8_t flag=1;
    // 初始化DWT计数器
    dwt_cnt_last = DWT->CYCCNT;
    
    // uint32_t last_time = HAL_GetTick();     // 用于计算dt
  /* Infinite loop */
  for(;;)
  {
        // 使用DWT计算高精度时间间隔
        dt = Dwt_GetDeltaT(&dwt_cnt_last);
       
        // 限制dt范围，防止异常值
        if (dt > 0.01f || dt <= 0.0f) {
            dt = 0.001f;  // 默认1ms
        }
        
        // 读取BMI088数据
        if (bmi088_test != NULL) {
            BMI088_Read(bmi088_test);
            
            // 读取温度数据并进行温度控制
            if (BMI088_ReadTemperature(bmi088_test->spi_acc, &temperature)) {
                bmi088_test->temperature = temperature;
                
                // 温度控制PID计算
                if (temp_pid != NULL && heater_pwm != NULL) {
                    float target_temp = 40.0f; // 目标温度40摄氏度
                    float pid_output = Pid_Calculate(temp_pid, target_temp, temperature);
                    
                    // 将PID输出转换为占空比
                    float duty_ratio = pid_output / 20.0f;
                    if (duty_ratio < 0.0f) duty_ratio = 0.0f;
                    if (duty_ratio > 1.0f) duty_ratio = 1.0f;
                    
                    // 设置PWM占空比
                    Pwm_SetDutyRatio(heater_pwm, duty_ratio);
                }
            }
            
            // 数据读取与预处理
            for (int i = 0; i < 3; i++) {
				filtered_accel[i] = bmi088_test->accel[i];
            }
        //    FusionAhrsFlags flags = FusionAhrsGetFlags(&fusion_ahrs);
        //     FusionAhrsInternalStates states = FusionAhrsGetInternalStates(&fusion_ahrs);

            Quater.ins_ready = (!fusion_ahrs.initialising); // 误差小于1度
            // 构造 Fusion 向量并进行动态偏置补偿
            FusionVector raw_gyro = {bmi088_test->gyro[0], bmi088_test->gyro[1], bmi088_test->gyro[2]};
            raw_gyro = FusionOffsetUpdate(&fusion_offset, raw_gyro); // 动态偏置学习与减除
            
            for (int i = 0; i < 3; i++) {
                filtered_gyro[i] = raw_gyro.array[i];
            }
            
            // 姿态解算 - 使用EKF进行姿态融合更新
            if (ins_initialized) {
                // 使用EKF进行高精度姿态解算（融合陀螺仪和加速度计数据）
                float compensated_accel[3];
                compensate_centrifugal_accel(filtered_gyro, filtered_accel, dt, compensated_accel);
                
                // 使用 Fusion AHRS 更新姿态（已修改库底层支持弧度）
                FusionVector fusion_gyro = {filtered_gyro[0], filtered_gyro[1], filtered_gyro[2]};
                FusionVector fusion_accel = {compensated_accel[0]/9.80665f, compensated_accel[1]/9.80665f, compensated_accel[2]/9.80665f};
                
                FusionAhrsUpdateNoMagnetometer(&fusion_ahrs, fusion_gyro, fusion_accel, dt);

                // 从 Fusion AHRS 获取四元数
                FusionQuaternion fq = FusionAhrsGetQuaternion(&fusion_ahrs);
                current_quaternion[0] = fq.element.w;
                current_quaternion[1] = fq.element.x;
                current_quaternion[2] = fq.element.y;
                current_quaternion[3] = fq.element.z;

                // 获取弧度制欧拉角
                FusionEuler fe = FusionQuaternionToEulerRad(fq);
                Quater.roll = fe.angle.roll;
                Quater.pitch = fe.angle.pitch;
                Quater.yaw = fe.angle.yaw;

                // // 使用补偿后的加速度更新 EKF
                // IMU_QuaternionEKF_Update(filtered_gyro[0], filtered_gyro[1], filtered_gyro[2], 
                //                         compensated_accel[0], compensated_accel[1], compensated_accel[2], dt);
                // IMU_QuaternionEKF_Update(filtered_gyro[0], filtered_gyro[1], filtered_gyro[2], 
                //                         filtered_accel[0], filtered_accel[1], filtered_accel[2], dt);
                
                // // 从EKF获取最优估计的四元数
                // current_quaternion[0] = QEKF_INS.q[0]; // w
                // current_quaternion[1] = QEKF_INS.q[1]; // x  
                // current_quaternion[2] = QEKF_INS.q[2]; // y
                // current_quaternion[3] = QEKF_INS.q[3]; // z
                // 从Madgwick获取最优估计的四元数
                // current_quaternion[0] = ahrs->q0; // w
                // current_quaternion[1] = ahrs->q1; // x
                // current_quaternion[2] = ahrs->q2; // y
                // current_quaternion[3] = ahrs->q3; // z
                
                // 更新全局四元数结构体
                // Quater.roll = QEKF_INS.Roll;     // 横滚角（度）
                // Quater.pitch = QEKF_INS.Pitch;   // 俯仰角（度）
                Quater.Gyro[0]=filtered_gyro[0];
                Quater.Gyro[1]=filtered_gyro[1];
                Quater.Gyro[2]=filtered_gyro[2];
                // 从四元数直接计算弧度制 yaw
                //float yaw_rad = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]), 
                 //                   2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) - 1.0f);
                //Quater.yaw = yaw_rad;  // 赋值给结构体（弧度制）
                // Quater.yaw = QEKF_INS.Yaw;   
								//-flag*(0.015*(bmi088_test->temperature-25)+1)*dt							// 偏航角（度）
                Quater.Acc.A_x = filtered_accel[0];
                Quater.Acc.A_y = filtered_accel[1];
                Quater.Acc.A_z = filtered_accel[2];
				///////////测试代码开始//////////////
							////////////测试代码结束//////////////
                // 可选：备用的四元数积分方法（用于对比或故障恢复）
                // caculate_angle(filtered_gyro, current_quaternion, current_quaternion, dt);
            }
        }
        
        // 读取磁力计数据
        //IstRead_mem(asdf, test_data);
        
        // 调试输出 - 每1000次循环输出一次
        // static uint32_t debug_counter = 0;
        // if (++debug_counter >= 1000) {
        //     // 检查BMI088校准状态
        //     uint8_t cali_status = BMI088_GetCalibrationStatus(bmi088_test);
            
        //     // 检查EKF是否收敛
        //     if (get_ekf_status()) {
        //         // EKF已收敛
        //         if (!cali_status) {
        //             // 校准未完成，可能是问题所在
        //         }
        //     } else {
        //         // EKF未收敛
        //         if (!cali_status) {
        //             // 校准未完成可能导致EKF无法收敛
        //         }
        //     }
        //     debug_counter = 0;
        // }
        
        osDelay(1); // 1ms延时，保持1000Hz更新频率
  }
  /* USER CODE END isttask */
}


/**
* @brief 初始化姿态（东北天坐标系）
*
* @param gyro 陀螺仪输出数值[3x1]
* @param origin_quater 初始姿态四元数[w,x,y,z]
* @param acc 加速度输出数值[3x1]
* @param magne 磁力计输出值[3x1]

* @return 0: 成功, -1: 失败, 1:妙妙成功
*/
uint8_t Quater_Init(float* origin_quater, uint8_t check) {
    extern Bmi088Instance_s *bmi088_test;
    
    if(check == 1) { 
        float32_t g0[3] = {0,0,0};
        float32_t g1[3] = {0,0,0};
        
        
        for(uint8_t i = 0; i < 50; i++){  // 从100改为50次
            BMI088_Read(bmi088_test);
            
            g0[0] += bmi088_test->accel[0];
            g0[1] += bmi088_test->accel[1];
            g0[2] += bmi088_test->accel[2];
            Gyro_Offset[0] += bmi088_test->gyro[0];
            Gyro_Offset[1] += bmi088_test->gyro[1];
            Gyro_Offset[2] += bmi088_test->gyro[2];
            osDelay(5);  
            
            BMI088_Read(bmi088_test);
            Gyro_Offset[0] += bmi088_test->gyro[0];
            Gyro_Offset[1] += bmi088_test->gyro[1];
            Gyro_Offset[2] += bmi088_test->gyro[2];
            g1[0] += bmi088_test->accel[0];
            g1[1] += bmi088_test->accel[1];
            g1[2] += bmi088_test->accel[2];
            osDelay(2);  
        }

        for (uint8_t i = 0; i < 3; ++i){
            g1[i] /= 50;  // 对应平均值除数

            g0[i] /= 50;
            Gyro_Offset[i] /= 100; //陀螺仪零偏
        }

    if(calculate_quaternion_from_gravity(g0,g1,origin_quater)<0){//此处即完成四元数初始化
      //处理失败情况
      Error_Handler();
    };
    //初始化Madgwick
    //beta，滤波器增益
    //sampleFreq，采样频率
    madgwick_ahrs=pvPortMalloc(sizeof(MadgwickAHRS));
    MadgwickAHRS_init(madgwick_ahrs, 0.1f, 1000.0f);

    //初始化Fusion AHRS - 必须先Initialise再SetSettings，否则会被覆盖
    FusionAhrsInitialise(&fusion_ahrs);
    FusionAhrsSetSettings(&fusion_ahrs, &settings);
    FusionOffsetInitialise(&fusion_offset, 1000); // 1000Hz 动态校准
    fusion_offset.gyroscopeOffset.axis.x = Gyro_Offset[0];
    fusion_offset.gyroscopeOffset.axis.y = Gyro_Offset[1];
    fusion_offset.gyroscopeOffset.axis.z = Gyro_Offset[2];
    
    FusionQuaternion init_q = {
        .element = {origin_quater[0], origin_quater[1], origin_quater[2], origin_quater[3]}
    };
    FusionAhrsSetQuaternion(&fusion_ahrs, init_q);

   // IMU_QuaternionEKF_Init(origin_quater,10, 0.001, 1000000,1,0);
    //初始化PID
  
    
    //以下为磁场转动...我是说，磁场校准部分，施工中
    // Ist8310_Init_Config_s ist8310_conf = {
    //   .gpio_conf_exti = {
    //       .exti_mode = GPIO_EXTI_MODE_FALLING,
    //       .GPIO_Pin = GPIO_PIN_3,
    //       .GPIOx = GPIOG,
    //       .gpio_model_callback = NULL,
    //   },
    //   .gpio_conf_rst = {
    //       .exti_mode = GPIO_EXTI_MODE_NONE,
    //       .GPIO_Pin = GPIO_PIN_6,
    //       .GPIOx = GPIOG,
    //       .gpio_model_callback = NULL,
    //   },
    //   .iic_config = {
    //       .handle = &hi2c3,
    //       .dev_address = IST8310_IIC_ADDRESS,
    //       .work_mode = IIC_BLOCK_MODE,
    //   },
    // };
    //注意，这类的注册都必须在实时系统外完成。
    //asdf = Ist8310Init(&ist8310_conf);
//    float32_t magnes[3]= {0, 0, 0};
//    for(uint8_t i = 0; i < 2; i++){
////        magnes[i] = asdf->mag[i];
//    }
//    //angle = atan2(magnes[1], magnes[0]);//通过磁力计计算偏航角的方式
 
    }else{
        origin_quater[0]=1;
        origin_quater[1]=0;
        origin_quater[2]=0;
        origin_quater[3]=0;//假设初始姿态就和坐标系对齐
     //初始化EKF
    //IMU_QuaternionEKF_Init(origin_quater,10, 0.001, 10000000,1,0);
     //MadgwickAHRS_init(madgwick_ahrs,0.1f, 0.001f);
        FusionAhrsSetSettings(&fusion_ahrs, &settings);
        FusionAhrsInitialise(&fusion_ahrs);
    }
    
    return 1;
    
}
//这之后的代码基本上都是已经被封装在内的，但是为了方便拓展我依然写在这里，不用担心占用内存

/**
* @brief 四元数乘法 q1 * q2
* @param q1 第一个四元数 [w, x, y, z]
* @param q2 第二个四元数 [w, x, y, z]  
* @param result 结果四元数 [w, x, y, z]
*/
void quaternion_multiply(const float* q1, const float* q2, float* result)
{
    float w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
    
    result[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;  // w
    result[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;  // x
    result[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;  // y
    result[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;  // z
}
/**
* @brief 四元数转欧拉角
* @param quaternion 输入四元数 [w, x, y, z]
* @param roll 输出横滚角 (弧度)
* @param pitch 输出俯仰角 (弧度)
* @param yaw 输出偏航角 (弧度)
* @author Claude Sonnet4
*/
void quaternion_to_euler(const float* quaternion, float* roll, float* pitch, float* yaw)
{
    float32_t w = quaternion[0], x = quaternion[1], y = quaternion[2], z = quaternion[3];
    
    // Roll (x轴旋转)
    float32_t sinr_cosp = 2.0f * (w * x + y * z);
    float32_t cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (y轴旋转)
    float32_t sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(M_PI / 2.0f, sinp);  // 万向节锁情况
    } else {
        *pitch = asinf(sinp);
    }
    
    // Yaw (z轴旋转)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

//计算模长（的平方）（只对xyz有效）
float calculate_norm(const float *arr){
        float arr_norm;
        arm_sqrt_f32(arr[0]*arr[0] + arr[1]*arr[1] + arr[2]*arr[2], &arr_norm);//求根号
    return arr_norm;
    
}
/**
* @brief 根据加速度计数据计算横滚角(roll)和俯仰角(pitch)
*
* @param g0 初始重力向量 [3x1]
* @param g1 当前重力向量 [3x1]
* @param quaternion 输出四元数 [w, x, y, z]
* @return 0: 成功, -1: 失败, 1:妙妙成功
*/
uint8_t calculate_quaternion_from_gravity( float32_t* g0,  float32_t* g1, float32_t* quaternion)
{
    float32_t g0_norm, g1_norm;
    float32_t g0_normalized[3], g1_normalized[3];
  float32_t cross_product[3];
  float32_t dot_product;
    
    
    
    //归一化g0和g1
    g0_norm=calculate_norm(g0);
    
    g1_norm=calculate_norm(g1);

    
    // 检查向量是否为零向量    
  if (g0_norm < 1e-6f || g1_norm < 1e-6f) {        
        return -1; //表示失败
  }
       // 归一化向量
    arm_scale_f32(g0, 1.0f/g0_norm, g0_normalized, 3);
    arm_scale_f32(g1, 1.0f/g1_norm, g1_normalized, 3);
    
    // 计算向量点积（cosθ）
      arm_dot_prod_f32(g0_normalized, g1_normalized, 3, &dot_product);//点积，因为是归一化的，所以输出就直接是cosθ值
    
    // 限制数值范围
    dot_product = fmaxf(-1.0f, fminf(1.0f, dot_product));
      //根据cos值反解θ
        acosf(dot_product);
    
    // 检查向量是否已经对齐（这个AI写的，我不清楚为什么要加进来）
    if (dot_product > 0.999999f) {
        quaternion[0] = 1.0f;  // w
        quaternion[1] = 0.0f;  // x
        quaternion[2] = 0.0f;  // y
        quaternion[3] = 0.0f;  // z
        return 0;
    }
    
    // 处理相反向量的情况（180度旋转）
    if (dot_product < -0.999999f) {
        // 找一个垂直轴进行180度旋转
        float32_t axis[3];
        if (fabsf(g0_normalized[0]) < 0.9f) {
            axis[0] = 1.0f; axis[1] = 0.0f; axis[2] = 0.0f;
        } else {
            axis[0] = 0.0f; axis[1] = 1.0f; axis[2] = 0.0f;
        }
        
        // 正交化处理
        float32_t dot_temp;
        arm_dot_prod_f32(axis, g0_normalized, 3, &dot_temp);  // 计算投影
        float32_t temp_vec[3];
        arm_scale_f32(g0_normalized, dot_temp, temp_vec, 3);  // 计算投影向量
        arm_sub_f32(axis, temp_vec, axis, 3);                // 去除投影分量
        
        // 归一化正交轴
        float32_t axis_norm;
        arm_power_f32(axis, 3, &axis_norm);
        arm_sqrt_f32(axis_norm, &axis_norm);
        arm_scale_f32(axis, 1.0f/axis_norm, axis, 3);
        
        // 构造180度旋转的四元数
        quaternion[0] = 0.0f;     // w = cos(π/2) = 0
        quaternion[1] = axis[0];  // x分量
        quaternion[2] = axis[1];  // y分量
        quaternion[3] = axis[2];  // z分量
        return 0;
        }
        // 计算叉积
    cross_product[0] = g0_normalized[1]*g1_normalized[2] - g0_normalized[2]*g1_normalized[1];
    cross_product[1] = g0_normalized[2]*g1_normalized[0] - g0_normalized[0]*g1_normalized[2];
    cross_product[2] = g0_normalized[0]*g1_normalized[1] - g0_normalized[1]*g1_normalized[0];
    
    // 计算四元数
    quaternion[0] = sqrtf((1.0f + dot_product) / 2.0f);  // w（用cos反解θ）
    float32_t w_inv = 1.0f / (2.0f * quaternion[0]);//方便后面计算
        
    //计算四元数（根据w计算因子（好高大上的名字））
    quaternion[1] = cross_product[0]*w_inv;  // x
    quaternion[2] = cross_product[1]*w_inv;  // y
    quaternion[3] = cross_product[2]*w_inv;  // z
        

        return 1;
}

/**
* @brief 四元数归一化
* @param quaternion 输入输出四元数 [w, x, y, z]
*/
void quaternion_normalize(float* quaternion)
{
    float norm_squared = quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1] + 
                        quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3];
    
    if (norm_squared < 1e-6f) {
        // 设置为单位四元数
        quaternion[0] = 1.0f;
        quaternion[1] = quaternion[2] = quaternion[3] = 0.0f;
        return;
    }
    
    float norm;
    arm_sqrt_f32(norm_squared, &norm);
    arm_scale_f32(quaternion, 1.0f/norm, quaternion, 4);
}

/**
* @brief 对陀螺仪进行积分来获取四元数角度更新（一阶龙格库塔法）
*
* @param gyro 陀螺仪输出数值[3x1]
* @param origin_quater 初始姿态四元数[4x1]
* @param dt 运行间隔
* @param quaternion 输出四元数 [w, x, y, z]
* @return 0: 成功, -1: 失败, 1:妙妙成功
*/
uint8_t caculate_angle(const float* gyro, const float* origin_quater, float* quaternion, float dt)
{
    // 检查输入参数
    if (gyro == NULL || origin_quater == NULL || quaternion == NULL || dt <= 0) {
        return -1;
    }
    
    // 构造角速度四元数 [0, wx, wy, wz]
    float omega_q[4] = {0.0f, gyro[0], gyro[1], gyro[2]};
    float temp_q[4];//导数
        float delta_q[4];
    // 计算四元数导数：dq/dt = 0.5 * w*q （右乘，作用于物体坐标系）
  
    quaternion_multiply(origin_quater, omega_q, temp_q);//qw
    
    // 除以二
    arm_scale_f32(temp_q, 0.5f, temp_q, 4);
    
    // 一阶积分：q(t+dt) = q(t) + dq/dt * dt
    
    arm_scale_f32(temp_q, dt, delta_q, 4);
    arm_add_f32(origin_quater, delta_q, quaternion, 4);
    
    // 归一化保持单位四元数性质
    quaternion_normalize(quaternion);
        
    
    return 1;
}