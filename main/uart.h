#ifndef _UART_H
#define _UART_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"  // freertos系统文件
#include "freertos/task.h"  // 任务创建
#include "driver/uart.h"  // 串口驱动
#include "driver/gpio.h"  // GPIO(引脚，输入输出)
#include "sdkconfig.h"
#include "string.h"

#define ECHO_TEST_TXD (GPIO_NUM_32)  // ESP32的发送引脚
#define ECHO_TEST_RXD (GPIO_NUM_33)  // ESP32的接收引脚
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)  // 不改变该引脚功能，不使用硬件控制流
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)  // 不改变该引脚功能，不使用硬件控制流
#define ECHO_UART_PORT_NUM 2  // uart的端口号
#define ECHO_UART_BAUD_RATE 9600  // uart的波特率
#define ECHO_TASK_STACK_SIZE 1024  // 任务堆栈大小
#define BUF_SIZE (1024)  // uart缓冲区大小
#define CONFIG_UART_ISR_IN_IRAM 0  // 用于设置UART驱动程序的中断处理函数是否存储在IRAM中，0即不存储，1即存储 

// 对数据类型进行宏定义
typedef unsigned char uint8_t;
typedef uint8_t  u8;
typedef unsigned short uint16_t;
typedef uint16_t  u16;
typedef unsigned int uint32_t;
typedef uint32_t  u32;

// 函数声明
void Uart_Init(void);
void Uart_Handle(void);

#endif