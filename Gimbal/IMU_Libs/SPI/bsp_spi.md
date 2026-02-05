# bsp_spi的使用文档

## 概述
bsp_spi是一个用于控制SPI总线的底层驱动程序，提供了基本的SPI通信功能，支持阻塞模式、中断模式和DMA模式。

## 开发环境
- 硬件平台：STM32F4系列(RoboMasterC板)
- 开发工具：Keil MDK-ARM
- 相关库：CMSIS, HAL

## 硬件说明
C型开发板引出两路SPI，分别连接BMI088或作为扩展IO在8pin牛角座引出。单个SPI总线上最多可挂载4个从机。

## 结构体定义

### 枚举定义
```c
typedef enum
{
    SPI_BLOCK_MODE = 0, // 默认使用阻塞模式
    SPI_IT_MODE,        // 中断模式
    SPI_DMA_MODE,       // DMA模式
} SPI_TXRX_MODE_e;
```

### SPI实例结构体
```c
typedef struct spi_ins_temp
{
    SPI_HandleTypeDef *spi_handle; // SPI外设handle
    GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等

    SPI_TXRX_MODE_e spi_work_mode; // 传输工作模式
    uint8_t rx_size;               // 本次接收的数据长度
    uint8_t *rx_buffer;            // 本次接收的数据缓冲区
    uint8_t CS_State;              // 片选信号状态,用于中断模式下的片选控制
    uint8_t * cs_pin_state;        // 片选信号状态,用于中断模式下的片选控制
    void (*callback)(struct spi_ins_temp *); // 接收回调函数
    void *id;                                // 模块指针
} SPIInstance;
```

### SPI初始化配置结构体
```c
typedef struct
{
    SPI_HandleTypeDef *spi_handle; // SPI外设handle
    GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等

    SPI_TXRX_MODE_e spi_work_mode; // 传输工作模式

    void (*callback)(SPIInstance *); // 接收回调函数
    void *id;                        // 模块指针
} SPI_Init_Config_s;
```

## 主要API

### SPI实例注册函数
```c
SPIInstance *SPIRegister(SPI_Init_Config_s *conf)
```
- 功能：注册一个SPI实例
- 参数：SPI初始化配置结构体指针
- 返回值：SPI实例指针，注册成功返回实例指针，否则返回NULL

### SPI数据发送函数
```c
void SPITransmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len)
```
- 功能：SPI发送数据
- 参数：
  - spi_ins：SPI实例指针
  - ptr_data：待发送数据指针
  - len：数据长度
- 说明：发送前会自动拉低片选信号，阻塞模式下发送完成后会自动拉高片选

### SPI数据接收函数
```c
void SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len)
```
- 功能：SPI接收数据
- 参数：
  - spi_ins：SPI实例指针
  - ptr_data：接收数据缓冲区指针
  - len：数据长度
- 说明：接收前会自动拉低片选信号，阻塞模式下接收完成后会自动拉高片选

### SPI同时收发函数
```c
void SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len)
```
- 功能：SPI同时发送和接收数据
- 参数：
  - spi_ins：SPI实例指针
  - ptr_data_rx：接收数据缓冲区指针
  - ptr_data_tx：待发送数据指针
  - len：数据长度
- 说明：该函数会同时进行数据发送和接收，需要确保发送和接收的缓冲区大小相同

### SPI工作模式设置函数
```c
void SPISetMode(SPIInstance *spi_ins, SPI_TXRX_MODE_e spi_mode)
```
- 功能：设置SPI工作模式
- 参数：
  - spi_ins：SPI实例指针
  - spi_mode：SPI工作模式
- 说明：可以在运行时切换SPI工作模式（阻塞/中断/DMA）

## 使用示例

### 1. 基本使用流程
```c
#include "bsp_spi.h"

// 定义回调函数
void my_spi_callback(SPIInstance *spi)
{
    // 处理接收到的数据
    uint8_t *data = spi->rx_buffer;
    uint8_t size = spi->rx_size;
    
    // 用户处理逻辑
}

void spi_example(void)
{
    // 1. 创建并初始化SPI配置
    SPI_Init_Config_s spi_config = {
        .spi_handle = &hspi1,           // SPI句柄
        .GPIOx = GPIOA,                 // 片选GPIO
        .cs_pin = GPIO_PIN_4,           // 片选引脚
        .spi_work_mode = SPI_BLOCK_MODE, // 使用阻塞模式
        .callback = my_spi_callback,    // 回调函数
        .id = NULL                      // 不使用id指针
    };
    
    // 2. 注册SPI实例
    SPIInstance *my_spi = SPIRegister(&spi_config);
    
    // 3. 准备数据缓冲区
    uint8_t tx_data[4] = {0x01, 0x02, 0x03, 0x04};
    uint8_t rx_data[4] = {0};
    
    // 4. 发送数据
    SPITransmit(my_spi, tx_data, 4);
    
    // 5. 接收数据
    SPIRecv(my_spi, rx_data, 4);
    
    // 6. 同时发送和接收数据
    SPITransRecv(my_spi, rx_data, tx_data, 4);
    
    // 7. 切换为DMA模式
    SPISetMode(my_spi, SPI_DMA_MODE);
    
    // 8. 在DMA模式下收发数据
    SPITransRecv(my_spi, rx_data, tx_data, 4);
    // 注意：DMA模式下数据处理在回调函数中进行
}
```

### 2. 多个从机的使用示例
```c
#include "bsp_spi.h"

// 定义两个SPI实例
SPIInstance *spi_device1;
SPIInstance *spi_device2;

// 回调函数
void device1_callback(SPIInstance *spi) {
    // 处理设备1的数据
}

void device2_callback(SPIInstance *spi) {
    // 处理设备2的数据
}

void multi_device_example(void)
{
    // 配置设备1
    SPI_Init_Config_s config1 = {
        .spi_handle = &hspi1,
        .GPIOx = GPIOA,
        .cs_pin = GPIO_PIN_4,
        .spi_work_mode = SPI_IT_MODE,
        .callback = device1_callback,
        .id = NULL
    };
    
    // 配置设备2
    SPI_Init_Config_s config2 = {
        .spi_handle = &hspi1,  // 同一SPI总线
        .GPIOx = GPIOB,        // 不同片选GPIO
        .cs_pin = GPIO_PIN_12,
        .spi_work_mode = SPI_IT_MODE,
        .callback = device2_callback,
        .id = NULL
    };
    
    // 注册设备
    spi_device1 = SPIRegister(&config1);
    spi_device2 = SPIRegister(&config2);
    
    // 使用设备1
    uint8_t tx_data[4] = {0x01, 0x02, 0x03, 0x04};
    uint8_t rx_data[4] = {0};
    SPITransmit(spi_device1, tx_data, 4);
    
    // 使用设备2
    uint8_t tx_data2[2] = {0xA0, 0xA1};
    uint8_t rx_data2[2] = {0};
    SPITransmit(spi_device2, tx_data2, 2);
}
```

## 注意事项
1. 请确保在CubeMX中已正确配置SPI引脚和时钟，并生成对应的初始化代码。
2. 在DMA和中断模式下，需要确保回调函数正确实现，否则无法获取接收到的数据。
3. 使用同一SPI总线的不同设备需要使用不同的片选信号（CS引脚）。
4. 在中断和DMA模式下，应确保rx_buffer在回调函数被调用前保持有效，否则可能导致未定义行为。
5. 单个SPI总线最多支持4个从机，如需更多可修改`MX_SPI_BUS_SLAVE_CNT`宏定义。

## 相关知识
SPI的通信过程如下： 
1. 主设备将要进行通讯的从设备的SS/CS片选拉低， 
2. 主设备通过SCK向从设备提供同步通讯所需要的时钟信号， 
3. 主设备通过MOSI向从设备发送8位数据，同时通过MISO接收从设备发来的8位数
据。 
4. 通信结束，主设备拉高SS/CS片选。