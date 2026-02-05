/**
 * @file dev_Ist_8310.c
 * @brief 用于读取IST8310传感器数据的驱动程序。
 * @author CGH and HWX
 * 
 * @date 2025-7-5
 * @note 他妈的磁场转动，一百万匹力量！
 */
#include "dev_ist_8310.h"//不要用中文引号！
#include <string.h>//memset函数
#include "FreeRTOS.h"  // 用于malloc、free等内存分配函数
#include <stdio.h>
//#define I2C_DMA_ENABLE 0 // 是否启用I2C DMA功能

Ist8310_Init_Config_s ISTConfig;

//以下是简单延时函数
void delay_us(uint32_t us) {
    volatile uint32_t count; // volatile防止编译器优化
    for (count = 0; count < us * 10; count++); // 调整系数适配主频
}

// 简单毫秒级延时
void delay_ms(uint32_t ms) {
    while (ms--) {
        delay_us(1000); // 调用微秒级延时
    }
}

// -------------------初始化写入数组,只使用一次,详见datasheet-------------------------
// the first column:the registers of IST8310. 第一列:IST8310的寄存器
// the second column: the value to be writed to the registers.第二列:需要写入的寄存器值
// the third column: return error value.第三列:返回的错误码
static uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01},  // enalbe interrupt  and low pin polarity.中断寄存器，配置成开启中断，中断时为低电平；
    {0x41, 0x09, 0x02},  // average 2 times.采样次数寄存器,配置平均采样两次
    {0x42, 0xC0, 0x03},  // must be 0xC0. 必须是0xC0
    {0x0A, 0x0B, 0x04}}; // 200Hz output rate.200Hz输出频率
/**
 * 测量模式： 
0: 休眠模式 
1: 单次测量模式 
11: 连续测量模式输出频率200Hz
 */

/**
 * @brief 解码IST8310传感器函数
 * 
 * @param iic IIC实例指针
 * @note 该回调函数用于解码IST8310传感器的数据。EXIT和RST引脚分别指的是：
 * 
 * - MAG_EXTI引脚：用于外部中断触发，通常连接到GPIO的输入引脚上。
 * - MAG_RST引脚：用于复位IST8310传感器，通常连接到GPIO的输出引脚上。
 * 
 * @attention 该函数利用了小端储存的性质，才能直接复制不需要移位，移植时候切记
 */
static void IST8310Decode(IICInstance *iic){
    int16_t temp[3];                                     // 用于存储解码后的数据
    Ist8310Instance_s *ist8310 = (Ist8310Instance_s *)(iic->id); // iic的id保存了IST8310实例的指针(父指针)
//使用指针访问，可以直接修改到外部的实例
    memcpy(temp, ist8310->iic_buffer, 6 * sizeof(uint8_t)); // 不要强制转换,直接cpy
    for (uint8_t i = 0; i < 3; i++){
        ist8310->mag[i] = (float)temp[i] * MAG_SEN; // 乘以灵敏度转换成uT(微特斯拉)
    }

}

/**
 * @brief EXTI中断回调函数,说明DRDY拉低.主机启动传输并在结束后调用IST8310Decode进行数据解析
 * @note  注意IICAccessMem是阻塞的
 *
 * @param gpio 发生中断的GPIO实例
 */
static void IST8310StartTransfer(GPIOInstance *gpio)
{
    // 先获取IST8310实例的指针(通过gpio实例的父指针id)
    Ist8310Instance_s *ist_for_transfer = (Ist8310Instance_s *)gpio->id;//疑问：为何要间接访问IST实例？
    // 中断说明ist已经准备好读取数据寄存器;6个字节（数据是从0x03-0x08，6个字节的寄存器全都是磁场数据（磁场转动！）,读取后会进入IST8310Decode函数
    #ifdef I2C_DMA_ENABLE
    // 如果启用了DMA功能，则使用DMA传输
    #endif
    #ifndef I2C_DMA_ENABLE
    // 如果没有启用DMA功能，则使用阻塞方式传输数据
    // 读取6个字节的磁场数据寄存器
    IICAccessMem(ist_for_transfer->iic, IST8310_DATA_REG, ist_for_transfer->iic_buffer, 6, IIC_READ_MEM, 1);
    #endif
    // 传输完成后会进入IST8310Decode函数进行数据解析
    IST8310Decode(ist_for_transfer->iic);
}

Ist8310Instance_s *Ist8310Init(Ist8310_Init_Config_s *config){

    uint8_t who_am_i = 0;//检测用
    Ist8310Instance_s *ist8310 = (Ist8310Instance_s *)pvPortMalloc(sizeof(Ist8310Instance_s));
    memset(ist8310, 0, sizeof(Ist8310Instance_s));//清除垃圾值
    if (ist8310 == NULL) {
        // 处理内存分配失败
        return NULL;
    }
    // 初始化IST8310实例
    config->iic_config.id= config->gpio_conf_exti.id = config->gpio_conf_rst.id = ist8310;//全部子实例的id都指向父实例（作用暂时不清楚，待查）
    config->iic_config.callback = IST8310Decode;//设置IIC回调函数
		
    config->gpio_conf_exti.gpio_model_callback = IST8310StartTransfer;
    //TODO:GPIO回调函数

    // 注册两个GPIO和IIC
    ist8310->iic = IICRegister(&config->iic_config);
    ist8310->gpio_exti = GPIORegister(&config->gpio_conf_exti);
    ist8310->gpio_rst = GPIORegister(&config->gpio_conf_rst);
    //进行重置
    GPIOReset(ist8310->gpio_rst);
    //HAL_Delay(10);//出现了卡死在HAL_Delay的问题，遂改用软件定时器
    delay_ms(sleepTime);
    GPIOSet(ist8310->gpio_rst);
    delay_ms(sleepTime);

    //检查IST8310 ID保证正确
    IICAccessMem(ist8310->iic, IST8310_WHO_AM_I, &who_am_i, 1, IIC_READ_MEM, 1);
    if (who_am_i != 0X10){//跃鹿的这个define意义很差...没啥好说的，直接0X01算了
        return NULL; 
    }
   
    //写入配置  TODO:将跃鹿的矩阵式寄存器配置数据改为结构体式，更清晰（内存占用也更大）
    // 进行初始化配置写入并检查是否写入成功,这里用循环把最上面初始化数组的东西都写进去
    for (uint8_t i = 0; i < IST8310_WRITE_REG_NUM; i++)
    { // 写入配置,写一句就读一下看看ist8310是否仍然在线
        IICAccessMem(ist8310->iic, ist8310_write_reg_data_error[i][0], &ist8310_write_reg_data_error[i][1], 1, IIC_WRITE_MEM, 1);
        IICAccessMem(ist8310->iic, ist8310_write_reg_data_error[i][0], &who_am_i, 1, IIC_READ_MEM, 1); // 读回自身id
        if (who_am_i != ist8310_write_reg_data_error[i][1]){
            while (1)
                printf("[ist8310] init error, code %d", ist8310_write_reg_data_error[i][2]); // 掉线/写入失败/未知错误,会返回对应的错误码

        }
    }

    return ist8310;
}


///////////////////////调试用代码部分//////////////////////////////
void IstRead_mem(Ist8310Instance_s *ist8310,uint8_t test[5]){
    IICAccessMem(ist8310->iic, IST8310_WHO_AM_I, &test[0], 1, IIC_READ_MEM, 1);
    IICAccessMem(ist8310->iic, IST8310_INTERRUPT_REG, &test[1], 1, IIC_READ_MEM, 1);
    IICAccessMem(ist8310->iic, IST8310_STATUS_REG, &test[2], 1, IIC_READ_MEM, 1);
    IICAccessMem(ist8310->iic, IST8310_GPIO_REG, &test[3], 1, IIC_READ_MEM, 1);
   
}

//TODO:DWT计时器以实现更精确的中断