#include <stdio.h>
#include "freertos/FreeRTOS.h"  // freertos系统文件
#include "freertos/task.h"  // 任务创建
#include "freertos/queue.h"  // 引入FreeRTOS操作系统中队列相关头文件
#include "driver/gpio.h"  // 定义了GPIO相关函数和数据类型
#include "driver/i2c.h"  // ESP32芯片上使用I2C总线驱动
#include "esp_log.h"  // 定义了ESP-IDF中日志输出相关的函数和数据类型
#include "esp_attr.h"   // 指定存储位置
#include "nvs.h"  // 定义了ESP32芯片上的非易失性存储器（NVS，Non-Volatile Storage）相关的函数和数据类型
#include "nvs_flash.h"  // 定义了ESP32芯片上的NVS部分的初始化和擦除操作函数，以及相关的数据类型
#include "mcpwm.h"  // PWM驱动头文件
#include "PCA9685.h"  // PWM扩展板驱动头文件
#include "robot_state.h"  // 机器人驱动头文件
#include "uart.h"  // 串口驱动头文件
#include "lvgl_init.h"  // lvgl图形库驱动文件
#include "pub.h"  // 系统时间驱动文件
#include "mpu_dmp_driver.h"  // 陀螺仪驱动头文件
#include "can.h"  // can协议驱动头文件
#include <math.h>  // 调用数学函数头文件
#include <string.h>
#include <stdint.h>
#include <unistd.h> 
#include <stdlib.h>

#define TAG "MAIN"
#define GPIO_MPU_INTR 23

extern float pitch, roll, yaw, temp;
static xQueueHandle gpio_evt_queue = NULL; // 用于接收 GPIO 中断的 queue  

// 初始化所有需要使用的GPIO 
void gpio_init(void);
void gpio_task(void* arg);  
void gpio_intr_handle(void* arg);

void merge();

// 主函数
void app_main(void)
{
    /*************************机器人行走测试用例*************************/
    // // 定义需要使用的变量
    // // double t, angles[LEG_DOF];  // t是当前时间, angles[LEG_DOF]存储了当前舵机应当处于的角度
    // // int steps = 100;  // 时间步长数
    // // double dt = Cycle_time / steps;  //dt是时间分辨率

    // // 初始化PCA9685的16组PWM
    // PCA9685_Init(50, 90);      

    // // 初始化成功
    // printf("Init Succeed!\n");

    // while(1)
	// {
    //     // 打印寄存器中的数据，进行测试
    //     printf("向PCA9685写数据的地址寄存器的数据为:%d\n", PCA9685_Read(PCA_Addr));
    //     printf("Model_1地址寄存器的数据为:%d\n", PCA9685_Read(PCA_Model));
    //     printf("PWM0寄存器的数据:%d\n", PCA9685_Read(LED0_OFF_L));
    //     printf("PWM1寄存器的数据:%d\n", PCA9685_Read(LED0_OFF_L + 1 * 4));
    //     printf("PWM2寄存器的数据:%d\n", PCA9685_Read(LED0_OFF_L + 2 * 4));

    //     // 设置PWM成功返回提示
    //     if(PCA9685_Read(PCA_Addr) == 0 && PCA9685_Read(PCA_Model) == 33)
    //     {
    //         printf("Connect PWM Succeed!\n");
    //     }
    //     else
    //     {
    //         printf("Connect PWM Failed!\n");
    //     }

    //     // 通过设置的时间周期来进行机器人步态的角度处理
    //     for (int i = 0; i < steps; i++) 
    //     {
    //         // 计算当前时间
    //         t = i * dt;

    //         // 计算机器人当前时间下应该处于的角度和运动状态
    //         Compute_Angles(t, angles);
    //         printf("第%d段时间步长的电机运动角度是: ", i + 1); 
            
    //         // 设置当前角度到PWM中
    //         Set_Angles_To_PWM(angles);
            
    //         //进行时间延时设置来满足模拟机器人的步态运动时间
    //         usleep(dt * 1000000);
    //     }
	// }
    /*************************************************************************/


    /****************************新电路板机器人行走避障测试用例*****************************/
    // // 定义需要使用的变量
    // double t, angles[LEG_DOF];  // t是当前时间, angles[LEG_DOF]存储了当前舵机应当处于的角度
    // int steps = 100;  // 时间步长数
    // double dt;  //dt是时间分辨率
    // char str[20];  // 输出到串口的转换字符串
    // double times;  // 转动次数

    // // 初始化PCA9685输出PWM角度
    // PCA9685_Init(50, 90);
    // // 初始化串口
    // Uart_Init();

    // while(1)
	// {
    //     // 向上位PC发送一次数据处理申请
    //     uart_write_bytes(ECHO_UART_PORT_NUM, "1111lettry", strlen("1111lettry"));  

    //     // 接收处理PC通过串口发送的数据
    //     uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    //     // 获取所接收数据的长度，若未接收到消息，则阻塞程序150ms
    //     int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE, 500 / portTICK_RATE_MS);  
    //     // 对接收到的数据进行处理，得到需要转向的角度
    //     // uart_write_bytes(ECHO_UART_PORT_NUM, data, len); 
    //     double angle = strtod((char*)data, NULL);
    //     sprintf(str, "Angle is %.2f\n", angle);
    //     uart_write_bytes(ECHO_UART_PORT_NUM, str, strlen(str));
        
    //     // 机器人转一定的角度
    //     dt = Cycle_time_Turn / steps;  

    //     // 对转向角度进行判断，进一步确定是左转还是右转
    //     if(angle > 0.0)
    //     {
    //         // 计算左转次数
    //         times = round(angle / 4.5);

    //         // 开始左转
    //         for(int j = 0; j < (int) times; j++)
    //         {
    //             for (int i = 0; i < steps; i++) 
    //             {
    //                 // 计算当前时间
    //                 t = i * dt;
                
    //                 // 机器人左转
    //                 Robot_Turn_Left(t, angles);
                
    //                 //进行时间延时设置来满足模拟机器人的步态运动时间
    //                 usleep(dt * 1000000);
    //             }
    //         }
    //     }
    //     else
    //     {
    //         // 计算右转次数
    //         times = round(fabs(angle) / 4.5);

    //         // 开始右转
    //         for(int j = 0; j < (int) times; j++)
    //         {
    //             for (int i = 0; i < steps; i++) 
    //             {
    //                 // 计算当前时间
    //                 t = i * dt;
                
    //                 // 机器人右转
    //                 Robot_Turn_Right(t, angles);
                
    //                 //进行时间延时设置来满足模拟机器人的步态运动时间
    //                 usleep(dt * 1000000);
    //             }
    //         }
    //     }

    //     // 发送测试输出
    //     sprintf(str, "Time is %.2f", times);
    //     uart_write_bytes(ECHO_UART_PORT_NUM, str, strlen(str));

    //     // 直走
    //     dt = Cycle_time_Walk / steps;  

    //     // 通过设置的时间周期来进行机器人步态的角度处理
    //     for (int i = 0; i < steps; i++) 
    //     {
    //         // 计算当前时间
    //         t = i * dt;
            
    //         // 机器人直走
    //         Robot_Walk(t, angles);
            
    //         //进行时间延时设置来满足模拟机器人的步态运动时间
    //         usleep(dt * 1000000);
    //     }

    //     // 进行串口数据处理
    //     // Uart_Handle();

    //     // 释放内存，防止内存泄露
    //     free(data); 
    // }
    /*************************************************************************/


    /***************************ESP32mcpwm测试用例****************************/
    // 自定义要使用的变量
    // int i;
    // int ANGLE_2 = 180;

    // // 初始化电机驱动
    // InitServo();

    // // 主循环
    // while(1)
    // {
    //     // // 延迟250ms
    //     // vTaskDelay(250 / portTICK_RATE_MS);

    //     // // 循环18次,每次使电机转动10度,循环18次以后对角度进行清零重置
    //     // for(i = 0; i <= 18; i++)
    //     // {
    //     //     // 设置角度
    //     //     SetServoAngle(0, 'A', ANGLE_2);
    //     //     // 延迟500ms
    //     //     vTaskDelay(500 / portTICK_RATE_MS);
    //     //     ANGLE_2 -= 10;
    //     // }

    //     // // 清零角度重置
    //     // if(i == 19)
    //     // {
    //     //     i = 0;
    //     //     ANGLE_2 = 180;
    //     //     // 延迟1000ms
    //     //     vTaskDelay(1000 / portTICK_RATE_MS);
    //     // }

    //     //设置角度为180
    //     SetServoAngle(0, 'A', 0);
    //     // 延迟2000ms
    //     usleep(2000000);
    //     //设置角度为0
    //     SetServoAngle(0, 'A', 180);
    //     // 延迟2000ms
    //     usleep(2000000);
    // }
    /*************************************************************************/


    /************************ESP32显示屏和陀螺仪测试用例*************************/
    //nvs初始化，必须要用
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // can协议初始化
    Can_Init();

    // 陀螺仪MPU6050初始化
    mpu_dmp_init();

    // 相关引脚初始化
    gpio_init();

    // 创建队列接收
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    // 安装GPIO终端程序
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_MPU_INTR, gpio_intr_handle, (void*) GPIO_MPU_INTR);

    // 1.显示屏初始化
    xTaskCreatePinnedToCore(guiTask1, "gui", 4096*2, NULL, 1, NULL, 1);

    // 延时(等待显示屏初始化完成)
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // 提示系统启动成功
    DEBUG_CODE{ESP_LOGI(TAG,"System Started, Time:%ld",UpdateCurrentTime());};  // 打印系统启动时间

    //定义用于控制执行的时间变量
    long LastWriteLCDTime=0;
    long LastWriteBTTime=0;
    long LastDebugOutTime=0;
    while(1)
    {
        MainLoop++;

        //1.更新系统时间
        UpdateCurrentTime();

        //3.刷新显示屏的内容,注意，刷新时间不能太短，否则CPU处理不了,一般100ms刷新一次
        if(CurrentTime-LastWriteLCDTime>100)
        {
            LastWriteLCDTime=CurrentTime;
            char str1[100];
            char str2[100];

            sprintf(str1,"pitch: %f\nroll: %f\nyaw: %f\nax: %hd\nay: %hd\naz: %hd\n", pitch, roll, yaw, accel[0], accel[1], accel[2]);
            // sprintf(str1,"你好啊!");
            if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                lv_label_set_text(label_2, str1);//设定文本内容
                lv_label_set_text(label_2, str1);//设定文本内容
                xSemaphoreGive(xGuiSemaphore);
            }
        }

        //4.周期性调试输出，用于输出状态信息、统计信息等，一般1秒输出一次
        if(CurrentTime-LastDebugOutTime>1000)
        {
            LastDebugOutTime=CurrentTime;
            printf("测试输出：CurrentTime=%ld,MainLoop=%ld\n",CurrentTime,MainLoop);
        }

        // 通过can协议发送数据
        send();

        //5.主循环一定要有vTaskDelay，归还CPU，让执行其它系统底层处理
        // vTaskDelay(10 / portTICK_PERIOD_MS);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    /**************************************************************************/


    /****************************LED(GPIO15 | GPIO2 | GPIO4)测试用例****************************/
    // 设置LED灯的相关引脚的参数
	// {
    //     gpio_config_t io_conf;

    //     io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //     io_conf.pin_bit_mask = ((1ULL<<GPIO_NUM_15) | (1ULL<<GPIO_NUM_2) | (1ULL<<GPIO_NUM_4));
    //     io_conf.mode = GPIO_MODE_OUTPUT;
    //     io_conf.pull_up_en = 0;
    //     io_conf.pull_down_en = 0;
    //     gpio_config(&io_conf);
    // }

    // // 主循环
    // while(1)
    // {
    //     gpio_set_level(GPIO_NUM_2, 0);
    //     gpio_set_level(GPIO_NUM_4, 0);
    //     gpio_set_level(GPIO_NUM_15, 0);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    //     printf("GPIO_2 is %d\n", gpio_get_level(GPIO_NUM_2));
    //     printf("GPIO_4 is %d\n", gpio_get_level(GPIO_NUM_4));
    //     printf("GPIO_15 is %d\n", gpio_get_level(GPIO_NUM_15));
    //     gpio_set_level(GPIO_NUM_2, 1);
    //     gpio_set_level(GPIO_NUM_4, 1);
    //     gpio_set_level(GPIO_NUM_15, 0);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    //     printf("GPIO_2 is %d\n", gpio_get_level(GPIO_NUM_2));
    //     printf("GPIO_4 is %d\n", gpio_get_level(GPIO_NUM_4));
    //     printf("GPIO_15 is %d\n", gpio_get_level(GPIO_NUM_15));
    // }
    /**************************************************************************/
}

// 显示屏GPIO的引脚初始化（输入中断引脚）
void gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_MPU_INTR),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);
}

// GPIO的触发性任务
void gpio_task(void* arg)
{
    uint32_t io_num;
    int count=0;

    gpio_config_t io_conf = {
        //.pin_bit_mask = ((1ULL << 14)|(1ULL << 12)|(1ULL << 13)),
        .pin_bit_mask = (1ULL << 13),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_PIN_INTR_DISABLE,
    };
    gpio_config(&io_conf);


    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            gyro_data_ready_cb();
            dmp_get_data();
            // printf("pitch:%f,roll:%f,yaw:%f-------\n",pitch, roll, yaw);  // 测试使用,输出三个姿态角
            // printf("gx:%hd,gy:%hd,gz:%hd-------\n",gyro[0],gyro[1],gyro[2]);  // 测试使用,陀螺仪各轴的输出
            // printf("ax:%hd,ay:%hd,az:%hd-------\n",accel[0],accel[1],accel[2]);  // 测试使用,输出加速度原始数据
            usleep(100000);
        }
    }
}

void gpio_intr_handle(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
