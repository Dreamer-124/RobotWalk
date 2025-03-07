// 防止mcpwm.h文件被重复引入，若当前文件中没有引入，则引入此头文件
#ifndef _MCPWM_H
#define _MCPWM_H

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/gpio.h"  // GPIO引脚（输入输出）
#include "sdkconfig.h"  // Sdkconfig　配置选项

#define SERVO_MIN_PULSEWIDTH 500  // 最小脉冲宽度，以微秒为单位
#define SERVO_MAX_PULSEWIDTH 2500  // 最大脉冲宽度，以微秒为单位
#define SERVO_MAX_DEGREE 180  // 伺服电机可以转动的最大角度

// 定义PWM输出引脚
#define SERVO_GPIO1 18
#define SERVO_GPIO2 17
#define SERVO_GPIO3 27
#define SERVO_GPIO4 26

// 函数声明
void InitServo();
float servo_per_degree_init(uint32_t degree_of_rotation);
void SetServoAngle(int adress, char mode, int angle);

#endif