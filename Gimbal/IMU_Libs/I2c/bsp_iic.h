#ifndef BSP_IIC_H


#define BSP_IIC_H
#include "i2c.h"
#include "stdint.h"

#define IIC_DEVICE_CNT 2   // C板支持两个iic控制器
#define MX_IIC_SLAVE_CNT 8 // iic支持最大从机数量为8

/**
 * @file bsp_iic.h
 * @author zhan xile
 * @brief 对iic进行初始化
 * @version 0.1
 * @date 2025-07-2
 * 
 * @copyright Copyright (c) 2025
 * 
 */

// 定义IIC_Mem_Mode_e枚举类型
typedef enum {
    IIC_WRITE_MEM = 0,
    IIC_READ_MEM
} IIC_Mem_Mode_e;

 /* i2c 三种工作模式 */
typedef enum
{
    // 基本工作模式
    IIC_BLOCK_MODE = 0, // 阻塞模式
    IIC_IT_MODE,        // 中断模式
    IIC_DMA_MODE,       // DMA模式

} IIC_Work_Mode_e;
/* I2C MEM工作模式枚举,这两种方法都是阻塞 */

/*------------seq阻塞模式-------------------*/
typedef enum
{
    IIC_SEQ_RELEASE,    // 完成传输后释放总线占有权,这是默认的传输方式
    IIC_SEQ_HOLDON = 0, // 保持总线占有权不释放,只支持IT和DMA模式
} IIC_Seq_Mode_e;

/* I2C 初始化配置结构体+实例 */

typedef struct iic_temp_s
{
    I2C_HandleTypeDef *handle; // i2c handle
    uint8_t dev_address;       // 暂时只支持7位地址(还有一位是读写位)

    IIC_Work_Mode_e work_mode;             // 工作模式
    uint8_t *rx_buffer;                    // 接收缓冲区指针
    uint8_t rx_len;                        // 接收长度
    void (*callback)(struct iic_temp_s *); // 接收完成后的回调函数

    void *id; // 用于标识i2c instance
} IICInstance;

/* I2C 初始化结构体配置 */
typedef struct
{
    I2C_HandleTypeDef *handle;       // i2c handle
    uint8_t dev_address;             // 暂时只支持7位地址(还有一位是读写位),注意不需要左移
    IIC_Work_Mode_e work_mode;       // 工作模式
    void (*callback)(IICInstance *); // 接收完成后的回调函数
    void *id;                        // 用于标识i2c instance
} IIC_Init_Config_s;

IICInstance *IICRegister(IIC_Init_Config_s *conf);

void IICSetMode(IICInstance *iic, IIC_Work_Mode_e mode);//设置工作模式是阻塞、中断还是dma模式
void IICTransmit(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode);//iic的数据传输包括句柄，数据及大小，然后还有seq的两种模式
void IICReceive(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode);//iic的数据接收与上行同理
void IICAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, 
                 uint16_t size, IIC_Mem_Mode_e mem_mode, uint8_t mem8bit_flag);//访问寄存器和存储器进行数据读取和写入
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);//实例回调函数
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);//存储器回调函数

#endif // BSP_IIC_H