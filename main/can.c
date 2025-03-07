#include "can.h"

// 定义消息计数的全局变量
int NumofRecv=0;
int count = 0;

// 消息发送函数
void send()
{
    // 定义发送CAN信息的变量
    twai_message_t message;

    // 初始化CAN消息变量
    message.identifier = 0x123;
    message.extd = 0;
    message.data_length_code = 8;

    // 对发送的消息帧进行赋值
    for (int i = 0; i < 7; i++) {
        message.data[i] = i + 0x50;
    }
    message.data[7] = (count++) + 0x00;

    // 进行消息发送并判断消息是否发送成功
    if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK)
    {
        printf("Message queued for transmission\n");
    } 
    else 
    {
        printf("Failed to queue message for transmission\n");
    }
}

// 消息接收函数
void recv()
{
    twai_message_t message;
    int num=0;

    while (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
        num++;
    }

    if(num)
    {
        NumofRecv+=num;
        printf("Recv %d/%d\n", num,NumofRecv);
    }
}

// Can协议初始化函数
void Can_Init()
{
    // 进行can协议的配置初始化
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(12, 14, TWAI_MODE_NORMAL);  // TX RX MODLE
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


    // 设置接收过滤器
    f_config.acceptance_code = 0x123<<21;//  00010010011     0x123
    f_config.acceptance_mask = ~(0x7FF<<21) ;//0111 1111 1111   0x7FF
    f_config.single_filter = true;

    unsigned int value;
    value=0x7FF;
    printf("%x << 21: %x\n",value,(value << 21));

    
    value=0x123;
    printf("%x << 21: %x\n",value,(value << 21));


    value=0x55;
    printf("%x << 21: %x\n",value,(value << 21));


    // 进行Can协议驱动安装并判断是否成功
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    // 进行Can协议启动并判断是否成功
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }
}