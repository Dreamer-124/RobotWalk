#ifndef _IIC_H
#define _IIC_H

#include "freertos/FreeRTOS.h"  // freertos系统文件
#include "freertos/task.h"  // 任务创建
#include "esp_attr.h"   
#include "driver/gpio.h"  // gpio驱动库
#include "sdkconfig.h"  // Sdkconfig配置选项
#include "esp32/rom/ets_sys.h"  // esp32的系统库 
#include <unistd.h>  // 延时函数所用头文件   

// 对数据类型进行宏定义
typedef unsigned char uint8_t;
typedef uint8_t  u8;
typedef unsigned short uint16_t;
typedef uint16_t  u16;
typedef unsigned int uint32_t;
typedef uint32_t  u32;

// #define IIC_SDA GPIO_NUM_21  // 定义数据信号引脚
// #define IIC_SCL GPIO_NUM_22  // 定义时钟信号引脚

// // 宏定义SDA(数据信号传输)线的输入和输出模式
// #define SDA_IN {gpio_set_direction(IIC_SDA, GPIO_MODE_INPUT);}
// #define SDA_OUT {gpio_set_direction(IIC_SDA, GPIO_MODE_OUTPUT);}

// // 宏定义设置IIC_SDA引脚和IIC_SCL引脚输出高低电平
// #define IIC_SDA_HIGH {gpio_set_level(IIC_SDA, 1);}
// #define IIC_SCL_HIGH {gpio_set_level(IIC_SCL, 1);}
// #define IIC_SDA_LOW {gpio_set_level(IIC_SDA, 0);}
// #define IIC_SCL_LOW {gpio_set_level(IIC_SCL, 0);}

// #define READ_SDA (gpio_get_level(IIC_SDA))  // 读取数据信号线的电平信息

#define IIC_SDA GPIO_NUM_13  // 定义数据信号引脚
#define IIC_SCL GPIO_NUM_15  // 定义时钟信号引脚

// 宏定义SDA(数据信号传输)线的输入和输出模式
#define SDA_IN {gpio_set_direction(IIC_SDA, GPIO_MODE_INPUT);}
#define SDA_OUT {gpio_set_direction(IIC_SDA, GPIO_MODE_OUTPUT);}

// 宏定义设置IIC_SDA引脚和IIC_SCL引脚输出高低电平
#define IIC_SDA_HIGH {gpio_set_level(IIC_SDA, 1);}
#define IIC_SCL_HIGH {gpio_set_level(IIC_SCL, 1);}
#define IIC_SDA_LOW {gpio_set_level(IIC_SDA, 0);}
#define IIC_SCL_LOW {gpio_set_level(IIC_SCL, 0);}

#define READ_SDA (gpio_get_level(IIC_SDA))  // 读取数据信号线的电平信息

// 函数声明
void IIC_Init(void);                				 
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(u8 txd);			
u8 IIC_Read_Byte(unsigned char ack);
u8 IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);				
void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  

#endif