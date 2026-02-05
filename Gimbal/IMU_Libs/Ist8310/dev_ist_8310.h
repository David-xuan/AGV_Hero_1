#ifndef DEV_IST_8310_H
#define DEV_IST_8310_H



//#pragma once // 防止头文件重复包含,也可以用header guard

#include "bsp_iic.h" 

#include "bsp_gpio.h"//等待GPIO的BSP库实例定义
#include "stdint.h"

// 传感器灵敏度系数
#define MAG_SEN 0.3f // raw int16 data change to uT unit. 原始整型数据变成 单位ut
//各种寄存器地址
#define sleepTime 10 // 复位后等待10ms,使得IST8310稳定下来,可以读取数据
#define IST8310_DATA_REG  0x03       // ist8310的数据寄存器，0x03-0x08都是
#define IST8310_WHO_AM_I  0x00       // ist8310的ID寄存器
#define IST8310_STATUS_REG 0x02      // ist8310的状态寄存器
#define IST8310_INTERRUPT_REG  0x09       // ist8310的中断状态寄存器
#define IST8310_GPIO_REG 0x0B //ist8310的GPIO（配置）寄存器
// IST8310 ret ERROR CODE
#define IST8310_NO_ERROR 0x00    // seldom used. 一般不会用到
#define IST8310_NO_SENSOR 0x40   // seldom used. 一般不会用到
#define IST8310_IIC_ADDRESS 0x0E // the I2C slave address of IST8310
//寄存器数量
#define IST8310_WRITE_REG_NUM 4
/**
 * @brief IST8310 实例定义
 * @attention 配置GPIO Pin的时候注意使用GPIO_PIN_x(1,2,3,...,这是一个宏),而不是1,2,3的整形!
 *
 */
typedef struct tempist8310
{
    IICInstance *iic;        // iic实例
    GPIOInstance *gpio_rst;  // gpio实例,用于复位
    GPIOInstance *gpio_exti; // gpio实例,用于获取MAG_DRDY引脚状态,判断数据是否准备好(EXTI外部中断)
    uint8_t iic_buffer[8];   // iic接收缓冲区
    float mag[3];            // 三轴磁力计数据,[x,y,z]

   // void (*ist_module_callback)(IICInstance *); // 模块回调函数,感觉无用所以注释掉
    // 后续有需要再添加
    // ...
} Ist8310Instance_s;

/**
 * @brief IST8310 初始化配置结构体
 * @attention 配置GPIO Pin的时候注意使用GPIO_PIN_x(1,2,3,...,这是一个宏),而不是1,2,3的整形!
 *
 */
typedef struct
{
    IIC_Init_Config_s iic_config;      // iic初始化配置
    GPIO_Init_Config_s gpio_conf_rst;  // gpio初始化配置,用于复位ist8310,看数据手册
    GPIO_Init_Config_s gpio_conf_exti; // gpio初始化配置,用于获取MAG_DRDY引脚状态,判断数据是否准备好(EXTI外部中断)
} Ist8310_Init_Config_s;

/**
 * @brief IST8310 初始化.
 * @note  注意一条i2c总线只能挂载一个IST8310,他们的地址是固定的,不能改变.
 * 并且因为是自动的,所以不需要手动进行数据更新（也就是不用写IIC更新函数）
 * @param config 配置结构体指针,包含iic配置和gpio配置
 *
 */
Ist8310Instance_s *Ist8310Init(Ist8310_Init_Config_s *config);
void IstRead_mem(Ist8310Instance_s *ist8310,uint8_t test[5]);





#endif// End of file: dev_ist_8310.h