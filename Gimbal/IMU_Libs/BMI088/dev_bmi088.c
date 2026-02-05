/**
 * @file dev_bmi088.c
 * @brief BMI088传感器的实现文件
 * @author CGH
 * @date 2025-07-19
 */
#include "dev_bmi088.h"
#include "bsp_spi.h"
#include "string.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#define BMI088_GYRO_2000_SEN        0.00106526443603169529841533860381f  // 2000°/s
#include "math.h"  // 添加数学函数支持
/*!***************************************************
 * @file: BMI088.c
 * @brief: 通过SPI通讯总线读取和发送传感器数据
 * @author:   WHX（玺佬的代码）
 * @editor： CGH     
 * @date:2025/07/19
 * @note:	
 ****************************************************/
/**
 * @brief 读取BMI088加速度计寄存器
 * @param spi_ins SPI实例
 * @param reg 寄存器地址
 * @param data 读取到的数据存储指针
 * @param len 读取长度
 * @return true 读取成功, false 读取失败
 */
 bool BMI088_AccelRead(SpiInstance_s* spi_ins, uint8_t reg, uint8_t* data, uint8_t len) {
    if (spi_ins == NULL || data == NULL || len == 0) {
        return false;
    }
    
    uint8_t tx_buffer[8] = {0}; // 最多读取6字节数据 + 2字节dummy
    uint8_t rx_buffer[8] = {0};
    
    tx_buffer[0] = 0x80 | reg; // 读操作 | 寄存器地址
    // tx_buffer[1] 是dummy byte
    
    // 使用TransRecv进行读取 (发送命令同时接收数据)
    if (Spi_TransRecv(spi_ins, rx_buffer, tx_buffer, len + 2)) {
        // 加速度计需要跳过前2个字节 (命令字节 + dummy字节)
        memcpy(data, &rx_buffer[2], len);
        return true;
    }
    return false;
}

/**
 * @brief 读取BMI088陀螺仪寄存器
 * @param spi_ins SPI实例
 * @param reg 寄存器地址
 * @param data 读取到的数据存储指针
 * @param len 读取长度
 * @return true 读取成功, false 读取失败
 */
 bool BMI088_GyroRead(SpiInstance_s* spi_ins, uint8_t reg, uint8_t* data, uint8_t len) {
    if (spi_ins == NULL || data == NULL || len == 0) {
        return false;
    }
    
    uint8_t tx_buffer[7] = {0}; // 最多读取6字节数据 + 1字节dummy
    uint8_t rx_buffer[7] = {0};
    
    tx_buffer[0] = 0x80 | reg; // 读操作 | 寄存器地址
    
    // 使用TransRecv进行读取
    if (Spi_TransRecv(spi_ins, rx_buffer, tx_buffer, len + 1)) {
        // 陀螺仪需要跳过前1个字节 (命令字节)
        memcpy(data, &rx_buffer[1], len);
        return true;
    }
    return false;
}

/**
 * @brief 写入BMI088加速度计寄存器
 * @param spi_ins SPI实例
 * @param reg 寄存器地址
 * @param data 要写入的数据
 * @return true 写入成功, false 写入失败
 */
static bool BMI088_AccelWrite(SpiInstance_s* spi_ins, uint8_t reg, uint8_t data) {
    if (spi_ins == NULL) {
        return false;
    }
    
    uint8_t tx_buffer[2] = {reg, data}; // 寄存器地址 + 数据
    return Spi_Transmit(spi_ins, tx_buffer, 2);
}

/**
 * @brief 写入BMI088陀螺仪寄存器
 * @param spi_ins SPI实例
 * @param reg 寄存器地址
 * @param data 要写入的数据
 * @return true 写入成功, false 写入失败
 */
static bool BMI088_GyroWrite(SpiInstance_s* spi_ins, uint8_t reg, uint8_t data) {
    if (spi_ins == NULL) {
        return false;
    }
    
    uint8_t tx_buffer[2] = {reg, data}; // 寄存器地址 + 数据
    return Spi_Transmit(spi_ins, tx_buffer, 2);
}
/**
 * @brief 读取BMI088 WHO AM I寄存器示例
 * @param acc_spi 加速度计SPI实例
 * @param gyro_spi 陀螺仪SPI实例
 * @return true 读取成功且ID正确, false 读取失败或ID错误
 */
bool BMI088_CheckWhoAmI(SpiInstance_s* acc_spi, SpiInstance_s* gyro_spi) {
    uint8_t acc_id = 0;
    uint8_t gyro_id = 0;
    
    // 读取加速度计WHO AM I (地址0x00, 期望值0x1E)
    if (!BMI088_AccelRead(acc_spi, 0x00, &acc_id, 1)) {
        return false;
    }
    
    // 读取陀螺仪WHO AM I (地址0x00, 期望值0x0F)  
    if (!BMI088_GyroRead(gyro_spi, 0x00, &gyro_id, 1)) {
        return false;
    }
    
    // 检查ID是否正确
    return (acc_id == 0x1E && gyro_id == 0x0F);
}

/**
 * @brief 读取BMI088加速度计数据示例
 * @param acc_spi 加速度计SPI实例
 * @param accel_data 存储加速度数据的数组[3] (x,y,z)
 * @return true 读取成功, false 读取失败
 */
bool BMI088_ReadAccelData(SpiInstance_s* acc_spi, float* accel_data) {
    uint8_t raw_data[6] = {0}; // 6字节原始数据 (每轴2字节)
    
    // 读取加速度计数据寄存器 (从0x12开始连续读取6字节)
    if (!BMI088_AccelRead(acc_spi, 0x12, raw_data, 6)) {
        return false;
    }
    
    // 转换为实际值 (需要根据量程设置调整系数)
    float acc_scale = 6.0f / 32768.0f; // 假设±6g量程

    for (int i = 0; i < 3; i++) {
        int16_t raw_value = (int16_t)((raw_data[i*2+1] << 8) | raw_data[i*2]);
        accel_data[i] = raw_value * acc_scale * 9.8f; // 转换为m/s²
    }
    
    return true;
}

/**
 * @brief 读取BMI088陀螺仪数据示例
 * @param gyro_spi 陀螺仪SPI实例  
 * @param gyro_data 存储陀螺仪数据的数组[3] (x,y,z)
 * @return true 读取成功, false 读取失败
 */
bool BMI088_ReadGyroData(SpiInstance_s* gyro_spi, float* gyro_data) {
    uint8_t raw_data[8] = {0}; // 8字节原始数据 (从CHIP_ID开始读取)
    
    // 读取陀螺仪数据寄存器 (从0x00 CHIP_ID开始连续读取8字节，参考成功库的做法)
    if (!BMI088_GyroRead(gyro_spi, 0x00, raw_data, 8)) {
        return false;
    }
    
    // 检查CHIP_ID是否正确 (应该是0x0F)
    if (raw_data[0] != 0x0F) {
        return false;
    }
    
    // 转换为实际值 (数据在字节2-7，对应X,Y,Z轴)
    // float gyro_scale = 2000.0f / 32768.0f; // 假设±2000°/s量程
    
    
    for (int i = 0; i < 3; i++) {
        // 数据从第2字节开始 (跳过CHIP_ID和reserved byte)
        int16_t raw_value = (int16_t)((raw_data[i*2+3] << 8) | raw_data[i*2+2]);
        gyro_data[i] = raw_value * BMI088_GYRO_2000_SEN; // 转换为°/s
    }
    
    return true;
}
/**
 * @brief 陀螺仪零偏校准函数
 * @param bmi088_ins BMI088实例指针
 * @note 参考C-board-ins的校准流程，增加极差检测和自恢复
 */
void BMI088_CalibrateOffset(Bmi088Instance_s *bmi088_ins) {
    if (bmi088_ins == NULL) {
        return;
    }
    
    // 添加调试信息，确认函数被调用
    // 可以通过断点或LED指示等方式确认
    
    const uint16_t cali_times = 6000;  // 校准采样次数
    const float max_g_norm_diff = 0.5f;  // 最大重力模长差异
    const float max_gyro_diff = 0.15f;   // 最大陀螺仪差异
    const float max_gyro_offset = 0.01f; // 最大允许零偏
    
    float gyro_max[3], gyro_min[3];
    float g_norm_temp, g_norm_max, g_norm_min;
    float gyro_diff[3], g_norm_diff;
    float start_time = Dwt_GetTimeline_s();  // 使用DWT计时
    
    do {
        // 超时保护
        if (Dwt_GetTimeline_s() - start_time > 10.0f) {
            // 校准超时，使用默认值
            bmi088_ins->gyro_offset[0] = 0.0f;
            bmi088_ins->gyro_offset[1] = 0.0f;
            bmi088_ins->gyro_offset[2] = 0.0f;
            bmi088_ins->g_norm = 9.8f;
            bmi088_ins->temp_when_cali = 40.0f;
            bmi088_ins->accel_scale[0] = bmi088_ins->accel_scale[1] = bmi088_ins->accel_scale[2] = 1.0f;
            break;
        }
        
        // 初始化累加器
        bmi088_ins->g_norm = 0.0f;
        bmi088_ins->gyro_offset[0] = 0.0f;
        bmi088_ins->gyro_offset[1] = 0.0f;
        bmi088_ins->gyro_offset[2] = 0.0f;
        
        // 采样数据
        for (uint16_t i = 0; i < cali_times; i++) {
            // 直接读取加速度计原始数据 (参考成功库的方式)
            uint8_t accel_buf[6] = {0};
            if (BMI088_AccelRead(bmi088_ins->spi_acc, 0x12, accel_buf, 6)) {
                // 转换加速度计数据
                int16_t raw_temp;
                float accel_scale = 6.0f / 32768.0f; // ±6g量程
                
                raw_temp = (int16_t)((accel_buf[1] << 8) | accel_buf[0]);
                bmi088_ins->accel[0] = raw_temp * accel_scale * 9.8f;
                raw_temp = (int16_t)((accel_buf[3] << 8) | accel_buf[2]);
                bmi088_ins->accel[1] = raw_temp * accel_scale * 9.8f;
                raw_temp = (int16_t)((accel_buf[5] << 8) | accel_buf[4]);
                bmi088_ins->accel[2] = raw_temp * accel_scale * 9.8f;
                
                // 计算重力模长
                g_norm_temp = sqrtf(bmi088_ins->accel[0] * bmi088_ins->accel[0] +
                                   bmi088_ins->accel[1] * bmi088_ins->accel[1] +
                                   bmi088_ins->accel[2] * bmi088_ins->accel[2]);
                bmi088_ins->g_norm += g_norm_temp;
            }
            
            // 直接读取陀螺仪原始数据 (参考成功库的方式)
            uint8_t gyro_buf[8] = {0};
            if (BMI088_GyroRead(bmi088_ins->spi_gyro, 0x00, gyro_buf, 8)) {
                if (gyro_buf[0] == 0x0F) { // 检查CHIP_ID
                    // 转换陀螺仪数据
                    int16_t raw_temp;
                    float gyro_scale = BMI088_GYRO_2000_SEN; // ±2000°/s量程
                    
                    raw_temp = (int16_t)((gyro_buf[3] << 8) | gyro_buf[2]);
                    bmi088_ins->gyro[0] = raw_temp * gyro_scale;
                    bmi088_ins->gyro_offset[0] += bmi088_ins->gyro[0];
                    raw_temp = (int16_t)((gyro_buf[5] << 8) | gyro_buf[4]);
                    bmi088_ins->gyro[1] = raw_temp * gyro_scale;
                    bmi088_ins->gyro_offset[1] += bmi088_ins->gyro[1];
                    raw_temp = (int16_t)((gyro_buf[7] << 8) | gyro_buf[6]);
                    bmi088_ins->gyro[2] = raw_temp * gyro_scale;
                    bmi088_ins->gyro_offset[2] += bmi088_ins->gyro[2];
                }
            }
            
            // 记录数据极差
            if (i == 0) {
                g_norm_max = g_norm_min = g_norm_temp;
                for (uint8_t j = 0; j < 3; j++) {
                    gyro_max[j] = gyro_min[j] = bmi088_ins->gyro[j];
                }
            } else {
                if (g_norm_temp > g_norm_max) g_norm_max = g_norm_temp;
                if (g_norm_temp < g_norm_min) g_norm_min = g_norm_temp;
                
                for (uint8_t j = 0; j < 3; j++) {
                    if (bmi088_ins->gyro[j] > gyro_max[j]) gyro_max[j] = bmi088_ins->gyro[j];
                    if (bmi088_ins->gyro[j] < gyro_min[j]) gyro_min[j] = bmi088_ins->gyro[j];
                }
            }
            
            // 检查数据稳定性，如果差异过大则重新开始
            g_norm_diff = g_norm_max - g_norm_min;
            for (uint8_t j = 0; j < 3; j++) {
                gyro_diff[j] = gyro_max[j] - gyro_min[j];
            }
            
            if (g_norm_diff > max_g_norm_diff ||
                gyro_diff[0] > max_gyro_diff ||
                gyro_diff[1] > max_gyro_diff ||
                gyro_diff[2] > max_gyro_diff) {
                break; // 数据不稳定，重新校准
            }
            
            Dwt_Delay(0.0005f); // 500μs延时
        }
        
        // 计算平均值
        bmi088_ins->g_norm /= (float)cali_times;
        for (uint8_t i = 0; i < 3; i++) {
            bmi088_ins->gyro_offset[i] /= (float)cali_times;
        }
        
        // 读取校准时温度
        BMI088_ReadTemperature(bmi088_ins->spi_acc, &bmi088_ins->temp_when_cali);
        
        Dwt_Delay(0.005f); // 5ms延时后重试检查
        
    } while (g_norm_diff > max_g_norm_diff ||
             fabsf(bmi088_ins->g_norm - 9.8f) > 0.5f ||
             gyro_diff[0] > max_gyro_diff ||
             gyro_diff[1] > max_gyro_diff ||
             gyro_diff[2] > max_gyro_diff ||
             fabsf(bmi088_ins->gyro_offset[0]) > max_gyro_offset ||
             fabsf(bmi088_ins->gyro_offset[1]) > max_gyro_offset ||
             fabsf(bmi088_ins->gyro_offset[2]) > max_gyro_offset);
    
    // 根据校准结果计算加速度计比例系数
    float accel_scale_factor = 9.81f / bmi088_ins->g_norm;
    bmi088_ins->accel_scale[0] = accel_scale_factor;
    bmi088_ins->accel_scale[1] = accel_scale_factor;
    bmi088_ins->accel_scale[2] = accel_scale_factor;
    
    // 使能零偏校准并设置校准完成标志
    bmi088_ins->cali_offset = 1;
    bmi088_ins->calibration_done = 1;
}

/**
 * @brief 读取所有IMU数据
 */
/**
 * @brief 读取所有IMU数据（支持零偏校准）
 */
bool BMI088_Read(Bmi088Instance_s *bmi088_ins){
    if (bmi088_ins == NULL) {
        return false;
    }
    
    // 读取加速度数据
    if (!BMI088_ReadAccelData(bmi088_ins->spi_acc, bmi088_ins->accel)) {
        return false;
    }
    
    // 应用加速度比例系数
    bmi088_ins->accel[0] *= bmi088_ins->accel_scale[0];
    bmi088_ins->accel[1] *= bmi088_ins->accel_scale[1];
    bmi088_ins->accel[2] *= bmi088_ins->accel_scale[2];
    
    // 读取陀螺仪数据
    if (!BMI088_ReadGyroData(bmi088_ins->spi_gyro, bmi088_ins->gyro)) {
        return false;
    }
    
    // 应用陀螺仪零偏校准
    if (bmi088_ins->cali_offset) {
        bmi088_ins->gyro[0] -= bmi088_ins->gyro_offset[0];
        bmi088_ins->gyro[1] -= bmi088_ins->gyro_offset[1];
        bmi088_ins->gyro[2] -= bmi088_ins->gyro_offset[2];
    }
    
    return true;
}

/**
 * @brief 获取校准状态
 * @param bmi088_ins BMI088实例指针
 * @return 校准状态：1-已校准，0-未校准
 */
uint8_t BMI088_GetCalibrationStatus(Bmi088Instance_s *bmi088_ins) {
    if (bmi088_ins == NULL) {
        return 0;
    }
    return bmi088_ins->calibration_done;
}

/**
 * @brief 检查BMI088加速度计错误寄存器
 * @param acc_spi 加速度计SPI实例
 * @return true 无错误, false 有错误
 */
bool BMI088_CheckAccelError(SpiInstance_s* acc_spi) {
    uint8_t error_reg = 0;
    
    // 读取加速度计错误寄存器 (地址0x02)
    if (!BMI088_AccelRead(acc_spi, 0x02, &error_reg, 1)) {
        return false; // 读取失败
    }
    
    // 解析错误寄存器
    uint8_t error_code = (error_reg >> 2) & 0x07; // 提取bit[4:2]
    uint8_t fatal_err = error_reg & 0x01;          // 提取bit[0]
    
    // 打印错误信息
    if (error_code != 0 || fatal_err != 0) {
        //printf("BMI088加速度计错误寄存器(0x02): 0x%02X\n", error_reg);
        
        // 解析错误代码
        switch (error_code) {
            case 0x00:
                //printf("  错误代码: 无错误\n");
                break;
            case 0x01:
                //printf("  错误代码: 加速度计配置错误 (ACC_CONF寄存器数据无效)\n");
                break;
            default:
                //printf("  错误代码: 未知错误 (0x%02X)\n", error_code);
                break;
        }
        
        // 检查致命错误标志
        if (fatal_err) {
            //printf("  致命错误: 芯片不在工作状态 (需要重新上电或软复位)\n");
        }
        
        return false; // 有错误
    }
    
    return true; // 无错误
}






/**
 * @brief BMI088回调函数
 * @param instance SPI实例指针
 * @note 该函数可以用于处理接收到的数据或其他回调逻辑
 */
void Bmi088_Callback(SpiInstance_s* instance) {
    if (instance == NULL || instance->id == NULL) {
        return;
    }
    
    // 从SPI实例获取BMI088实例
    Bmi088Instance_s* bmi088_ins = (Bmi088Instance_s*)instance->id;
    
    // 设置数据就绪标志
    bmi088_ins->data_ready = 1;
    
    // 这里可以添加数据处理逻辑
    
}
/**
 * @brief BMI088初始化函数
 * @param instance BMI088实例指针
 * @return true 初始化成功, false 初始化失败
 */
bool Bmi088_Init(Bmi088Instance_s *instance) {
    if (instance == NULL || instance->spi_acc == NULL || instance->spi_gyro == NULL) {
        return false;
    }
    // 执行dummy读取触发I2C到SPI模式切换
    uint8_t dummy_data = 0;
    BMI088_AccelRead(instance->spi_acc, 0x00, &dummy_data, 1);
    Dwt_Delay(0.001); // 等待1ms
    // 1. 检查WHO AM I寄存器
    if (!BMI088_CheckWhoAmI(instance->spi_acc, instance->spi_gyro)) {
        return false;  // 传感器连接失败
    }
    
    // 2. 初始化加速度计
    // 软复位
    if (!BMI088_AccelWrite(instance->spi_acc, 0x7E, 0xB6)) {
        return false;
    }
    // 延时等待复位完成
   Dwt_Delay(0.002);
		
     // 配置加速度计工作模式为活跃模式
    if (!BMI088_AccelWrite(instance->spi_acc, 0x7C, 0x00)) {
        return false;
    }
    // 配置加速度计电源模式
    if (!BMI088_AccelWrite(instance->spi_acc, 0x7D, 0x04)) { // 启用加速度计
        return false;
    }

   

    // 校验电源模式配置
    uint8_t power_mode = 0;
    Dwt_Delay(0.01); // 等待配置生效
    if (!BMI088_AccelRead(instance->spi_acc, 0x7D, &power_mode, 1) ) {
        return false; // 电源模式配置校验失败
    }

    // 配置加速度计工作模式和带宽
    if (!BMI088_AccelWrite(instance->spi_acc, 0x40, 0xAB)) { // 正常模式，800Hz
        return false;
    }//默认OSR1，（0xAB)噪声较大.若要使用OSR4，修改为0xA8

    // 校验工作模式配置
    uint8_t acc_conf = 0;
    Dwt_Delay(0.01); // 等待配置生效
    if (!BMI088_AccelRead(instance->spi_acc, 0x40, &acc_conf, 1) || acc_conf != 0xAB) {
        return false; // 工作模式配置校验失败
    }

    // 配置加速度计量程为±6g
    if (!BMI088_AccelWrite(instance->spi_acc, 0x41, 0x01)) {
        return false;
    }
    // 校验量程配置
    uint8_t acc_range = 0;
    Dwt_Delay(0.01); // 等待配置生效
    if (!BMI088_AccelRead(instance->spi_acc, 0x41, &acc_range, 1) || acc_range != 0x01) {
        return false; // 量程配置校验失败
    }
    
    // 配置加速度计数据就绪中断 (参考成功库的配置)
    // 配置INT1引脚为推挽输出，低电平有效
    if (!BMI088_AccelWrite(instance->spi_acc, 0x53, 0x08)) { // INT1_IO_CTRL
        return false;
    }
    
    // 映射数据就绪中断到INT1
    if (!BMI088_AccelWrite(instance->spi_acc, 0x58, 0x04)) { // INT_MAP_DATA
        return false;
    }
    
    // 3. 初始化陀螺仪
    // 软复位
    if (!BMI088_GyroWrite(instance->spi_gyro, 0x14, 0xB6)) {
        return false;
    }
    
    // 延时等待复位完成
    Dwt_Delay(0.003);//单位秒
    
    // 配置陀螺仪量程为±2000°/s
    if (!BMI088_GyroWrite(instance->spi_gyro, 0x0F, 0x00)) {
        return false;
    }

    // 校验陀螺仪量程配置
    uint8_t gyro_range = 0;
    Dwt_Delay(0.01); // 等待配置生效
    if (!BMI088_GyroRead(instance->spi_gyro, 0x0F, &gyro_range, 1) || gyro_range != 0x00) {
        return false; // 陀螺仪量程配置校验失败
    }
    
    // 配置陀螺仪带宽
    if (!BMI088_GyroWrite(instance->spi_gyro, 0x10, 0x81)) { // 2000Hz, 230Hz带宽
        return false;
    }//0x81的截止频率更低，能够更好的屏蔽噪声。若要使用532Hz宽带的模式，改用0x80
    
    // 使能陀螺仪正常模式
    if (!BMI088_GyroWrite(instance->spi_gyro, 0x11, 0x00)) {
        return false;
    }
    
    // 配置陀螺仪数据就绪中断 (参考成功库的配置)
    // 使能数据就绪中断
    if (!BMI088_GyroWrite(instance->spi_gyro, 0x15, 0x80)) { // GYRO_INT_CTRL
        return false;
    }
    
    // 配置INT3引脚为推挽输出，低电平有效
    if (!BMI088_GyroWrite(instance->spi_gyro, 0x16, 0x01)) { // INT3_INT4_IO_CONF
        return false;
    }
    
    // 映射数据就绪中断到INT3
    if (!BMI088_GyroWrite(instance->spi_gyro, 0x18, 0x01)) { // INT3_INT4_IO_MAP
        return false;
    }
    
    // 初始化标定参数
    for (int i = 0; i < 3; i++) {
        instance->gyro_offset[i] = 0.0f;
        instance->accel_scale[i] = 1.0f;
        instance->accel[i] = 0.0f;
        instance->gyro[i] = 0.0f;
    }
    instance->temperature = 0.0f;
    instance->g_norm = 9.8f;
    instance->temp_when_cali = 40.0f;
    instance->calibration_done = 0;  // 初始化校准完成标志
    // 注意：cali_offset 将在 Bmi088_Register 函数中设置
    
    // 5. 设置初始化完成标志
    instance->init_status = 1;
    instance->data_ready = 0;
    
    return true;
}


/**
 * @brief 重复执行init函数直到超时或者成功
 * @param bmi088_cfg BMI088配置结构体指针
 * @return BMI088实例指针，失败返回NULL
 */
Bmi088Instance_s* Bmi088_Register(Bmi088InitConfig_s *bmi088_cfg){
    if (bmi088_cfg == NULL) {
        return NULL; 
    }
    Bmi088Instance_s *bmi088_ins = (Bmi088Instance_s *)pvPortMalloc(sizeof(Bmi088Instance_s));
    if(bmi088_ins == NULL){
        return NULL; 
    } 
     uint16_t time = 0;
    // 清零结构体
    memset(bmi088_ins, 0, sizeof(Bmi088Instance_s));
    bmi088_ins->spi_module_callback=Bmi088_Callback; // 设置回调函数

    /*这里注册SPI实例*/
    // 配置加速度计SPI实例
    SpiInitConfig_s acc_spi_config = {
        .spi_handle = bmi088_cfg->spi_acc_config.spi_handle,
        .GPIOx = bmi088_cfg->spi_acc_config.GPIOx,
        .cs_pin = bmi088_cfg->spi_acc_config.cs_pin,
        .spi_work_mode = bmi088_cfg->spi_acc_config.spi_work_mode,
        .spi_module_callback = Bmi088_Callback,
        .id = bmi088_ins, // 将BMI088实例作为父指针传递
        .tx_len = 8,      // 加速度计最大传输长度
        .rx_len = 8       // 加速度计最大接收长度
    };
    
    // 配置陀螺仪SPI实例
    SpiInitConfig_s gyro_spi_config = {
        .spi_handle = bmi088_cfg->spi_gyro_config.spi_handle,
        .GPIOx = bmi088_cfg->spi_gyro_config.GPIOx,
        .cs_pin = bmi088_cfg->spi_gyro_config.cs_pin,
        .spi_work_mode = bmi088_cfg->spi_gyro_config.spi_work_mode,
        .spi_module_callback = Bmi088_Callback,
        .id = bmi088_ins, // 将BMI088实例作为父指针传递
        .tx_len = 7,      // 陀螺仪最大传输长度
        .rx_len = 7       // 陀螺仪最大接收长度
    };
    
    // 注册SPI实例
    bmi088_ins->spi_acc = Spi_Register(&acc_spi_config);
    bmi088_ins->spi_gyro = Spi_Register(&gyro_spi_config);
    
    // 检查SPI注册是否成功
    if (bmi088_ins->spi_acc == NULL || bmi088_ins->spi_gyro == NULL) {
        vPortFree(bmi088_ins);
        return NULL; // SPI注册失败
    }
    
    // 传递校准配置给实例
    bmi088_ins->cali_offset = bmi088_cfg->enable_calibration;

    // 循环直到超时或者注册成功
    while(Bmi088_Init(bmi088_ins) != true ) {
        
        // 添加延时避免过快重试
				Dwt_Delay(0.1);
    }
    if (time >= 100) {
        // 初始化超时，释放内存
        vPortFree(bmi088_ins);
        return NULL;
    }
    
    // 初始化成功后，如果启用校准则执行校准
    if (bmi088_ins->cali_offset) {
        BMI088_CalibrateOffset(bmi088_ins);
    }
    
    // 设置其他实例参数
    bmi088_ins->init_status = 1;
    bmi088_ins->data_ready = 0;
    
    return bmi088_ins;
}



/**
 * @brief 读取BMI088温度数据
 * @param acc_spi 加速度计SPI实例 (温度传感器在加速度计部分)
 * @param temperature 存储温度数据的指针 (单位：摄氏度)
 * @return true 读取成功, false 读取失败
 */
bool BMI088_ReadTemperature(SpiInstance_s* acc_spi, float* temperature) {
    if (acc_spi == NULL || temperature == NULL) {
        return false;
    }
    
    uint8_t temp_data[2] = {0}; // 2字节温度数据
    
    // 读取温度寄存器 (从0x22开始连续读取2字节: TEMP_MSB, TEMP_LSB)
    if (!BMI088_AccelRead(acc_spi, 0x22, temp_data, 2)) {
        return false;
    }
    
    // 检查是否为无效数据 (TEMP_LSB = 0x80表示无效)
    if (temp_data[1] == 0x80) {
        return false; // 无效温度数据
    }
    
    
    // Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32)
    uint16_t temp_uint11 = (temp_data[0] * 8) + (temp_data[1] / 32);
    
    // 转换为11位有符号整数
    int16_t temp_int11;
    if (temp_uint11 > 1023) {
        temp_int11 = temp_uint11 - 2048;  // 2's complement conversion
    } else {
        temp_int11 = temp_uint11;
    }
    
    // 计算最终温度值
    // Temperature = Temp_int11 * 0.125°C/LSB + 23°C
    *temperature = temp_int11 * 0.125f + 23.0f;
    
    return true;
}