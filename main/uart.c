#include "uart.h"

/**********************************
 * 函数全称: 
 * void uartinit(void)
 * 
 * 函数作用:
 * 初始化uart
 * ************************************/
void Uart_Init(void)
{
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,  // uart波特率
        .data_bits = UART_DATA_8_BITS,  // uart数据位，此处是8位
        .parity    = UART_PARITY_DISABLE,  // 串口通信禁用奇偶校验，如果需奇偶校验，UART_PARITY_ODD（奇校验）或UART_PARITY_EVEN（偶校验）
        .stop_bits = UART_STOP_BITS_1,  // 停止位（此处是1）
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // 禁用硬件流控制
        .source_clk = UART_SCLK_APB,  // 使用APB时钟作为UART的时钟源，该时钟源的频率是CPU时钟频率的一半
    };
    int intr_alloc_flags = 0;

    // 如果CONFIG_UART_ISR_IN_IRAM被定义为1，表示将中断处理函数存储在IRAM中，将中断处理函数存储在IRAM中，可以提高串口通信的性能。
    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    // 设置UART参数并且检查函数执行时是否报错
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    // 设置串口引脚并且检查函数执行时是否报错
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
    // 安装UART驱动程序并且检查函数执行时是否报错
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
}

/**********************************
 * 函数全称: 
 * void Uart_Handle(void)
 * 
 * 函数作用:
 * 对uart接收到的数据进行处理并且发送消息到上位PC
 * ************************************/
void Uart_Handle(void)
{
    // 定义存储串口接收数据的字符数组
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    // 接收数据并且获取其长度
    int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);  
	
    // 如果串口接收数据的长度大于2，则发送信息至串口调试助手（注：中文会出现乱码的情况）
    if(len >= 2)  
    {    
        uart_write_bytes(ECHO_UART_PORT_NUM, data, len);      
        uart_write_bytes(ECHO_UART_PORT_NUM, "\nDate Recieve Succeed!", strlen("\nDate Recieve Succeed!"));  
    } 

    // 释放内存，防止内存泄露
    free(data); 
}

