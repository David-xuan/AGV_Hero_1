/**
*   @file bsp_spi.c
*   @brief SPI通讯的实现
*   @author Wenxin HU
*   @date 25-7-15
*   @version 0.1
*   @note
*/

#ifndef _BSP_SPI_H_
#define _BSP_SPI_H_

#include "spi.h"
#include "gpio.h"
#include <stdint.h>
#include <stdbool.h>

/* 根据开发板引出的spi引脚以及CubeMX中的初始化配置设定 */
#define SPI_DEVICE_CNT 2       // C型开发板引出两路spi,分别连接BMI088/作为扩展IO在8pin牛角座引出
#define MX_SPI_BUS_SLAVE_CNT 4 // 单个spi总线上挂载的从机数目

typedef enum {
    SPI_BLOCKING_MODE = 0, // 默认使用阻塞模式
    SPI_IT_MODE = 1,
    SPI_DMA_MODE = 2,
} SpiMode_e;

/* SPI实例结构体 */
#pragma pack(1)
typedef struct _SpiInstance_s {
    SPI_HandleTypeDef* spi_handle; // SPI外设handle
    GPIO_TypeDef* GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等

    SpiMode_e spi_work_mode; // 传输工作模式
    uint8_t tx_len;          // 本次发送的数据长度
    uint8_t tx_buffer[50];   // 本次发送的数据缓冲区
    uint8_t rx_len;          // 本次接收的数据长度
    uint8_t rx_buffer[50];   // 本次接收的数据缓冲区
    uint8_t CS_State;        // 片选信号状态,用于中断模式下的片选控制
    uint8_t* cs_pin_state;    // 片选信号状态,用于中断模式下的片选控制

    void (*spi_module_callback)(struct _SpiInstance_s*); // 接收回调函数
    void* id;                                            // 模块指针
} SpiInstance_s;

/* SPI实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct {
    SPI_HandleTypeDef* spi_handle; // SPI外设handle
    GPIO_TypeDef* GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等
    uint8_t tx_len;                // 本次发送的数据长度
    uint8_t rx_len;                // 本次接受的数据长度
    SpiMode_e spi_work_mode;       // 传输工作模式

    void (*spi_module_callback)(SpiInstance_s*); // 接收回调函数
    void* id;                                    // 模块指针
} SpiInitConfig_s;
#pragma pack()

/**
 * @brief SPI实例注册函数
 * @param config SPI实例初始化配置结构体指针
 * @return 注册成功返回SPI实例指针，如果注册失败则返回NULL
 */
SpiInstance_s* Spi_Register(SpiInitConfig_s* config);

/**
 * @brief SPI发送数据函数，使用SPI配置的参数
 * @param spi_ins SPI实例指针
 * @return
 */
bool Spi_Transmit(SpiInstance_s* spi_ins, uint8_t* data, uint8_t len);

/**
 * @brief SPI接收数据函数，使用SPI配置的参数
 * @param spi_ins SPI实例指针
 * @return true--接收成功   false--接收失败
 */
bool Spi_Recv(SpiInstance_s* spi_ins);

/**
 * @brief SPI发送接收函数，用于发送一包数据之后接收一包数据
 * @param spi_ins SPI实例指针
 * @param data_rx 接收数据指针
 * @param data_tx 发送数据指针
 * @param len 数据长度
 * @return true--发送接收成功   false--发送接收失败
 */
bool Spi_TransRecv(SpiInstance_s* spi_ins, uint8_t* data_rx, uint8_t* data_tx, uint8_t len);

/**
 * @brief 设置SPI工作模式
 * @param spi_ins SPI实例指针
 * @param spi_mode SPI工作模式
 * @return true--设置成功   false--设置失败
 */
bool Spi_SetMode(SpiInstance_s* spi_ins, SpiMode_e spi_mode);

#endif
