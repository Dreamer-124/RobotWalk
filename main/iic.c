#include "iic.h"

/**********************************
 * 函数全称: 
 * void IIC_Init(void)
 * 
 * 函数作用:
 * 初始化IIC
 * ************************************/
void IIC_Init(void)
{					     
	// 将IIC_SDA引脚和IIC_SCL引脚配置为输出	
	gpio_set_direction(IIC_SDA, GPIO_MODE_OUTPUT);
	gpio_set_direction(IIC_SCL, GPIO_MODE_OUTPUT);

	// 设置IIC_SDA引脚和IIC_SCL引脚输出高电平
	IIC_SDA_HIGH;
	IIC_SCL_HIGH;
}

/**********************************
 * 函数全称: 
 * IIC_Start(void)
 * 
 * 函数作用:
 * 产生IIC起始信号
 * ************************************/
void IIC_Start(void)
{
	// 设置SDA线输出模式
	SDA_OUT;     
	// 设置IIC_SDA引脚和IIC_SCL引脚输出高电平
	IIC_SDA_HIGH;
	IIC_SCL_HIGH;
	// 延迟4us
    usleep(4);  
	// 设置IIC_SDA引脚为低电平(STOP:when CLK is high DATA change form low to high)
	IIC_SDA_LOW;
	// 延迟4us
    usleep(4);
	// 设置IIC_SCL引脚为低电平,钳住IIC总线,准备发送或接收数据
	IIC_SCL_LOW;
}	  

/**********************************
 * 函数全称: 
 * void IIC_Stop(void)
 * 
 * 函数作用:
 * 产生IIC停止信号
 * ************************************/
void IIC_Stop(void)
{
	// 设置SDA线输出模式
	SDA_OUT;
	IIC_SCL_LOW;
	// 设置IIC_SDA引脚为低电平(STOP:when CLK is high DATA change form low to high)
	IIC_SDA_LOW;
	// 延迟4us
    usleep(4);
	IIC_SCL_HIGH;
	// 发送IIC总线结束信号
	IIC_SDA_HIGH;
	// 延迟4us
    usleep(4);
}

/**********************************
 * 函数全称: 
 * u8 IIC_Wait_Ack(void)
 * 
 * 函数作用:
 * 等待应答信号到来 (返回1，接收应答失败; 0,接收应答成功;)
 * ************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	// SDA设置为输入模式
	SDA_IN;    
	IIC_SDA_HIGH; 
	// 延迟2us
	usleep(2);  
	IIC_SCL_HIGH; 
	// 延迟2us
	usleep(2);

	while(READ_SDA)
	{
	 	ucErrTime++;
	 	if(ucErrTime>250)
	 	{
	 		IIC_Stop();
	 		return 1;
	 	}
	}

	IIC_SCL_LOW;  // 时钟线输出0; 
	return 0;  
} 

/**********************************
 * 函数全称: 
 * void IIC_Ack(void)
 * 
 * 函数作用:
 * 产生ACK应答
 * ************************************/
void IIC_Ack(void)
{
	IIC_SCL_LOW;
	SDA_OUT;
	IIC_SDA_LOW;
    // 延迟2us
	usleep(2);
	IIC_SCL_HIGH;
	// 延迟2us
	usleep(2);
	IIC_SCL_LOW;
}
	    
/**********************************
 * 函数全称: 
 * void IIC_NAck(void)
 * 
 * 函数作用:
 * 不产生ACK应答
 * ************************************/		
void IIC_NAck(void)
{
	IIC_SCL_LOW;
	SDA_OUT;
	IIC_SDA_HIGH;
	// 延迟2us
	usleep(2);
	IIC_SCL_HIGH;
	// 延迟2us
	usleep(2);
	IIC_SCL_LOW;
}					 				     

/**********************************
 * 函数全称: 
 * void IIC_Send_Byte(u8 txd)
 * 
 * 参数注解:
 * u8 txd(要发送的一个字节)
 * 
 * 函数作用:
 * IIC发送一个字节, 返回从机有无应答 (注: 返回1，有应答; 0，无应答;)
 * ************************************/
void IIC_Send_Byte(u8 txd)
{                        
	u8 t; 

	SDA_OUT; 	    
    IIC_SCL_LOW;  // 拉低时钟开始数据传输
    for(t = 0; t < 8; t++)
    {              
        // IIC_SDA = (txd & 0x80) >> 7;
		gpio_set_level(IIC_SDA, (txd & 0x80) >> 7);
        txd <<= 1; 
		// 对TEA5767这三个延时都是必须的	  
		// 延迟2us
		usleep(2);
		IIC_SCL_HIGH;
		// 延迟2us
		usleep(2);
		IIC_SCL_LOW;	
		// 延迟2us
		usleep(2);
    }
} 	    

/**********************************
 * 函数全称: 
 * u8 IIC_Read_Byte(unsigned char ack)
 * 
 * 参数注解:
 * unsigned char ack(是否产生ACK应答的标志)
 * 
 * 函数作用:
 * 读1个字节，ack=1时，发送ACK，ack=0，发送nACK (返回接收到的数据)
 * ************************************/
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	
	// SDA设置为输入
	SDA_IN;  
    for(i = 0; i < 8; i++ )
	{
        IIC_SCL_LOW; 
        // 延迟2us
		usleep(2);
		IIC_SCL_HIGH;
        receive <<= 1;
        
		if(READ_SDA)
		{
			receive++;
		}

		// 延迟1us
		usleep(1); 
    }

    if (!ack)
        IIC_NAck();  // 发送nACK
    else
        IIC_Ack();   // 发送ACK  

    return receive;
}

