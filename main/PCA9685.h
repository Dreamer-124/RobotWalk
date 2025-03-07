#ifndef __PCA9685_H
#define __PCA9685_H

#include "iic.h"
#include <math.h>
#include <unistd.h>  // 延时函数所用头文件

#define PCA_Addr 0x80  // 定义向PCA9685写数据的地址寄存器的地址
#define PCA_Model 0x00  // 定义MODEL1的地址

#define SERVO_MIN_DUTY_CYCLE 0.025  // 最小脉冲宽度，以微秒为单位
#define SERVO_MAX_DUTY_CYCLE 0.125  // 最大脉冲宽度，以微秒为单位
#define SERVO_MAX_DEGREE 180  // 伺服电机可以转动的最大角度
#define SERVO_EXTEND_BIT 4096  // PWM扩展板的分辨率

// 定义脉宽设置寄存器 (每一路PWM通道占用4个寄存器)
#define LED0_ON_L 0x06  
#define LED0_ON_H 0x07  
#define LED0_OFF_L 0x08  
#define LED0_OFF_H 0x09
#define PCA_Pre 0xFE

void PCA9685_Init(float hz,u8 angle);
void PCA9685_Write(u8 addr,u8 data);
u8 PCA9685_Read(u8 addr);
void PCA9685_setPWM(u8 num,u32 on,u32 off);
void PCA9685_setFreq(float freq);
void setAngle(u8 num,u8 angle);

#endif