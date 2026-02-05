# bsp_uart

## 注意事项
1. 只允许开启C板的UART1 UART6 和UART3（DBUS）
2. 请为每一个UART都打开中断，**如果你打开了DMA中断接收也要打开！！！**，尽量不要使用阻塞模式收发数据
3. DMA模式请一定打开循环模式（如DR16接收数据），库内只会在注册时开启一次DMA接收

## 参考例程
```C
//定义回调函数
void Uart3CallBack(UartInstance_s *uart_instance) {
    // 这里可以添加UART3的接收处理逻辑
    // 例如解析接收到的数据，或者将数据发送到其他模块
}
//定义cofig结构体
UartInitConfig_s uart3_config = {
    .uart_handle = &huart3,
    .mode = UART_DMA_MODE,
    .rx_len = 18,
    .tx_len = 0,
    .uart_module_callback = Uart3CallBack, // 这里可以设置回调函数
    .id = NULL // 如果不需要，可以设置为NULL
  };
  
//注册uart3实例
UartInstance_s *uart3_instance = Uart_Register(&uart3_config);
```