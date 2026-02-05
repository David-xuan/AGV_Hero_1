/**
*   @file bsp_spi.c
*   @brief 通讯的实现
*   @author Wenxin HU
*   @date 25-7-15
*   @version 0.1
*   @note
*/

#include "bsp_spi.h"
#include "FreeRTOS.h"
#include "string.h"

static SpiInstance_s* spi_instances[MX_SPI_BUS_SLAVE_CNT]; //SPI实例数组,用于存储SPI实例
static uint8_t idx = 0;
static uint8_t SPIDeviceOnGoing[SPI_DEVICE_CNT] = {1}; // 用于判断当前spi是否正在传输,防止多个模块同时使用一个spi总线 (0: 正在传输, 1: 未传输)

SpiInstance_s* Spi_Register(SpiInitConfig_s* config) {
    if (config == NULL || idx >= MX_SPI_BUS_SLAVE_CNT) {
        return NULL; // 如果空间已满或者没有配置信息，返回NULL
    }

    // 修复：检查相同SPI句柄 + 相同片选引脚的组合，而不是只检查SPI句柄
    for (uint8_t i = 0; i < idx; i++) { // 注意：这里应该是idx而不是SPI_DEVICE_CNT
        if (spi_instances[i] != NULL && 
            spi_instances[i]->spi_handle == config->spi_handle &&
            spi_instances[i]->GPIOx == config->GPIOx &&
            spi_instances[i]->cs_pin == config->cs_pin) {
            return NULL; // 如果相同的SPI句柄+片选组合已经注册过，返回NULL
        }
    }

    if (config->rx_len > 50 || config->tx_len > 50) {
        return NULL; // 检查发送接收缓存长度是否超过限制
    }

    SpiInstance_s* instance = (SpiInstance_s*)pvPortMalloc(sizeof(SpiInstance_s));
    if (instance == NULL) {
        return NULL; // 内存分配失败
    }
    
    memset(instance, 0, sizeof(SpiInstance_s));

    instance->spi_handle = config->spi_handle;
    instance->GPIOx = config->GPIOx;
    instance->cs_pin = config->cs_pin;
    instance->spi_work_mode = config->spi_work_mode;
    instance->spi_module_callback = config->spi_module_callback;
    instance->id = config->id;
    instance->tx_len = config->tx_len;
    instance->rx_len = config->rx_len;

    if (instance->spi_handle->Instance == SPI1) {
        instance->cs_pin_state = &SPIDeviceOnGoing[0];
    }
    else if (instance->spi_handle->Instance == SPI2) {
        instance->cs_pin_state = &SPIDeviceOnGoing[1];
    }
    else {
        while (1);
    }

    spi_instances[idx++] = instance;
    return instance;
}

bool Spi_Transmit(SpiInstance_s* spi_ins, uint8_t* data, uint8_t len) {
    if (spi_ins == NULL || data == NULL || len == 0) {
        return false; // 检查参数有效性
    }

    memcpy(spi_ins->tx_buffer,data,len);

    // 拉低片选,开始传输(选中从机)
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);

    switch (spi_ins->spi_work_mode) {
        case SPI_DMA_MODE:
            HAL_SPI_Transmit_DMA(spi_ins->spi_handle, data, len);
            return true;
        case SPI_IT_MODE:
            HAL_SPI_Transmit_IT(spi_ins->spi_handle, data, len);
            return true;
        case SPI_BLOCKING_MODE:
            HAL_SPI_Transmit(spi_ins->spi_handle, data, len, 1000); // 默认50ms超时
        // 阻塞模式不会调用回调函数,传输完成后直接拉高片选结束
            HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
            return true;
        default:
            while (1); // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况
            return false; // 不支持的模式
    }
}

bool Spi_Recv(SpiInstance_s* spi_ins) {
    if (spi_ins == NULL) {
        return false; // 检查参数有效性
    }

    // 拉低片选,开始传输
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    switch (spi_ins->spi_work_mode) {
        case SPI_DMA_MODE:
            HAL_SPI_Receive_DMA(spi_ins->spi_handle, spi_ins->rx_buffer, spi_ins->rx_len);
            return true;
        case SPI_IT_MODE:
            HAL_SPI_Receive_IT(spi_ins->spi_handle, spi_ins->rx_buffer, spi_ins->rx_len);
            return true;
        case SPI_BLOCKING_MODE:
            HAL_SPI_Receive(spi_ins->spi_handle, spi_ins->rx_buffer, spi_ins->rx_len, 1000);
        // 阻塞模式不会调用回调函数,传输完成后直接拉高片选结束
            HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
            return true;
        default:
            while (1); // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况
            return false;
    }
}

bool Spi_TransRecv(SpiInstance_s* spi_ins, uint8_t* ptr_data_rx, uint8_t* ptr_data_tx, uint8_t len) {
    // 用于稍后回调使用,请保证ptr_data_rx在回调函数被调用之前仍然在作用域内,否则析构之后的行为是未定义的!!!
    if (spi_ins == NULL || ptr_data_rx == NULL || ptr_data_tx == NULL || len == 0) {
        return false; // 检查参数有效性
    }

    spi_ins->rx_len = len;
    memset(spi_ins->rx_buffer, 0, len);
    // 等待上一次传输完成
    if (spi_ins->spi_handle->Instance == SPI1) {
        while (!SPIDeviceOnGoing[0]) {
        };
    }
    else if (spi_ins->spi_handle->Instance == SPI2) {
        while (!SPIDeviceOnGoing[1]) {
        };
    }
    // 拉低片选,开始传输
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    *spi_ins->cs_pin_state =
        spi_ins->CS_State =
        HAL_GPIO_ReadPin(spi_ins->GPIOx, spi_ins->cs_pin);
    switch (spi_ins->spi_work_mode) {
        case SPI_DMA_MODE:
            HAL_SPI_TransmitReceive_DMA(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len);
            return true;
        case SPI_IT_MODE:
            HAL_SPI_TransmitReceive_IT(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len);
            return true;
        case SPI_BLOCKING_MODE:
            HAL_SPI_TransmitReceive(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len, 1000); // 默认50ms超时
        // 阻塞模式不会调用回调函数,传输完成后直接拉高片选结束
            HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
            *spi_ins->cs_pin_state =
                spi_ins->CS_State =
                HAL_GPIO_ReadPin(spi_ins->GPIOx, spi_ins->cs_pin);
            return true;
        default:
            while (1); // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况
            return false;
    }
}

bool Spi_SetMode(SpiInstance_s* spi_ins, SpiMode_e spi_mode) {
    if (spi_mode != SPI_DMA_MODE && spi_mode != SPI_IT_MODE && spi_mode != SPI_BLOCKING_MODE)
        while (1); // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况
        return false;

    if (spi_ins->spi_work_mode != spi_mode) {
        spi_ins->spi_work_mode = spi_mode;
    }

    return true;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
    for (size_t i = 0; i < idx; i++) {
        // 如果是当前spi硬件发出的complete,且cs_pin为低电平(说明正在传输),则尝试调用回调函数
        if (spi_instances[i]->spi_handle == hspi && // 显然同一时间一条总线只能有一个从机在接收数据
            HAL_GPIO_ReadPin(spi_instances[i]->GPIOx, spi_instances[i]->cs_pin) == GPIO_PIN_RESET) {
            // 先拉高片选,结束传输,在判断是否有回调函数,如果有则调用回调函数
            HAL_GPIO_WritePin(spi_instances[i]->GPIOx, spi_instances[i]->cs_pin, GPIO_PIN_SET);
            *spi_instances[i]->cs_pin_state =
                spi_instances[i]->CS_State =
                HAL_GPIO_ReadPin(spi_instances[i]->GPIOx, spi_instances[i]->cs_pin);
            // @todo 后续添加holdon模式,由用户自行决定何时释放片选,允许进行连续传输
            if (spi_instances[i]->spi_module_callback != NULL) // 回调函数不为空, 则调用回调函数
                spi_instances[i]->spi_module_callback(spi_instances[i]);
            return;
        }
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    HAL_SPI_RxCpltCallback(hspi); // 直接调用接收完成的回调函数
}
