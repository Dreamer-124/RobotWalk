#ifndef __CAN_H__
#define __CAN_H__

#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 函数声明
void send();
void recv();
void Can_Init();

#endif