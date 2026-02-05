#include "bsp_iic.h"
#include "stdlib.h"
#include "string.h"
#include <stdint.h>
#include "FreeRTOS.h"


static uint8_t idx = 0; // 配合中断以及初始化
static IICInstance *iic_instance[IIC_DEVICE_CNT] = {NULL};
/**
 * @brief IIC初始化配置结构体
 * 
 */

IICInstance *IICRegister(IIC_Init_Config_s *conf)
{
    if (idx >= MX_IIC_SLAVE_CNT) // 超过最大实例数
        while (1)                // 酌情增加允许的实例上限,也有可能是内存泄漏
            ;
    // 申请到的空间未必是0, 所以需要手动初始化
    IICInstance *instance = (IICInstance *)pvPortMalloc(sizeof(IICInstance));
    memset(instance, 0, sizeof(IICInstance));

    instance->dev_address = conf->dev_address << 1; // 地址左移一位,最低位为读写位
    instance->callback = conf->callback;
    instance->work_mode = conf->work_mode;
    instance->handle = conf->handle;
    instance->id = conf->id;

    iic_instance[idx++] = instance;
    return instance;
}

void IICSetMode(IICInstance *iic, IIC_Work_Mode_e mode)
{
    // HAL库自带重入保护，无需额外处理
    if (iic->work_mode != mode) {
        iic->work_mode = mode;
    }
}

void IICTransmit(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode)
{
    // 参数检查
    if (seq_mode != IIC_SEQ_RELEASE && seq_mode != IIC_SEQ_HOLDON) {
        while (1); // 非法的序列传输模式
    }

    // 根据工作模式选择不同的传输方式
    switch (iic->work_mode) {
    case IIC_BLOCK_MODE:
        if (seq_mode != IIC_SEQ_RELEASE) {
            while (1); // 阻塞模式不支持HOLD_ON
        }
        HAL_I2C_Master_Transmit(iic->handle, iic->dev_address, data, size, 100);
        break;
        
    case IIC_IT_MODE:
        if (seq_mode == IIC_SEQ_RELEASE) {
            HAL_I2C_Master_Seq_Transmit_IT(iic->handle, iic->dev_address, 
                                          data, size, I2C_OTHER_AND_LAST_FRAME);
        } else {
            HAL_I2C_Master_Seq_Transmit_IT(iic->handle, iic->dev_address, 
                                          data, size, I2C_OTHER_FRAME);
        }
        break;
        
    case IIC_DMA_MODE:
        if (seq_mode == IIC_SEQ_RELEASE) {
            HAL_I2C_Master_Seq_Transmit_DMA(iic->handle, iic->dev_address, 
                                          data, size, I2C_OTHER_AND_LAST_FRAME);
        } else {
            HAL_I2C_Master_Seq_Transmit_DMA(iic->handle, iic->dev_address, 
                                          data, size, I2C_OTHER_FRAME);
        }
        break;
        
    default:
        while (1); // 未知的工作模式
    }
}

void IICReceive(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode)
{
    // 参数检查
    if (seq_mode != IIC_SEQ_RELEASE && seq_mode != IIC_SEQ_HOLDON) {
        while (1); // 非法的序列传输模式
    }

    // 保存接收缓冲区信息(用于回调函数)
    iic->rx_buffer = data;
    iic->rx_len = size;

    // 根据工作模式选择不同的接收方式
    switch (iic->work_mode) {
    case IIC_BLOCK_MODE:
        if (seq_mode != IIC_SEQ_RELEASE) {
            while (1); // 阻塞模式不支持HOLD_ON
        }
        HAL_I2C_Master_Receive(iic->handle, iic->dev_address, data, size, 100);
        break;
        
    case IIC_IT_MODE:
        if (seq_mode == IIC_SEQ_RELEASE) {
            HAL_I2C_Master_Seq_Receive_IT(iic->handle, iic->dev_address, 
                                        data, size, I2C_OTHER_AND_LAST_FRAME);
        } else {
            HAL_I2C_Master_Seq_Receive_IT(iic->handle, iic->dev_address, 
                                        data, size, I2C_OTHER_FRAME);
        }
        break;
        
    case IIC_DMA_MODE:
        if (seq_mode == IIC_SEQ_RELEASE) {
            HAL_I2C_Master_Seq_Receive_DMA(iic->handle, iic->dev_address, 
                                          data, size, I2C_OTHER_AND_LAST_FRAME);
        } else {
            HAL_I2C_Master_Seq_Receive_DMA(iic->handle, iic->dev_address, 
                                          data, size, I2C_OTHER_FRAME);
        }
        break;
        
    default:
        while (1); // 未知的工作模式
    }
}

void IICAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, 
                 uint16_t size, IIC_Mem_Mode_e mem_mode, uint8_t mem8bit_flag)
{
    uint16_t bit_flag = mem8bit_flag ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;
    
    switch (mem_mode) {
    case IIC_WRITE_MEM:
        HAL_I2C_Mem_Write(iic->handle, iic->dev_address, mem_addr, 
                          bit_flag, data, size, 100);
        break;
        
    case IIC_READ_MEM:
        HAL_I2C_Mem_Read(iic->handle, iic->dev_address, mem_addr, 
                         bit_flag, data, size, 100);
        break;
        
    default:
        while (1); // 非法的存储器访问模式
    }
}

/**
 * @brief I2C主模式接收完成回调函数
 * @param hi2c I2C硬件句柄
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // 遍历所有实例，找到匹配的实例并调用其回调函数
    for (uint8_t i = 0; i < idx; i++) {
        if (iic_instance[i]->handle == hi2c) {
            if (iic_instance[i]->callback != NULL) {
                iic_instance[i]->callback(iic_instance[i]);
            }
            return;
        }
    }
}

/**
 * @brief I2C存储器访问完成回调函数
 * @param hi2c I2C硬件句柄
 * @note 实际实现与HAL_I2C_MasterRxCpltCallback相同
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_MasterRxCpltCallback(hi2c);
}