#include "PCA9685.h"

/**********************************
 * 函数全称: 
 * void PCA9685_Init(float hz,u8 angle)
 * 
 * 参数注解:
 * float hz (要设置的频率)
 * 
 * u8 angle (要设置的角度)
 * 
 * 函数作用:
 * 初始化PCA9685模块
 * ************************************/
void PCA9685_Init(float hz,u8 angle)
{
	// 设置PCA9685寄存器的相关引脚的参数
	{
        gpio_config_t io_conf;

        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        io_conf.pin_bit_mask = ((1ULL<<GPIO_NUM_13) | (1ULL<<GPIO_NUM_15)  | (1ULL<<GPIO_NUM_12)| (1ULL<<GPIO_NUM_14)| (1ULL<<GPIO_NUM_16)| (1ULL<<GPIO_NUM_25)| (1ULL<<GPIO_NUM_26)| (1ULL<<GPIO_NUM_27));
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = 0;
        io_conf.pull_down_en = 0;
        gpio_config(&io_conf);
    }
    
    // 设置PCA9685寄存器的相关引脚低电平设置
    gpio_set_level(GPIO_NUM_12, 0);
    gpio_set_level(GPIO_NUM_14, 0);
    gpio_set_level(GPIO_NUM_16, 0);
    gpio_set_level(GPIO_NUM_25, 0);
    gpio_set_level(GPIO_NUM_26, 0);
    gpio_set_level(GPIO_NUM_27, 0);

	IIC_Init();
	PCA9685_Write(PCA_Model, 0x00);  // 对PCA9685进行重置
	PCA9685_setFreq(hz);
	
	// 初始化PCA9685输出PWM使舵机旋转至90度
	PCA9685_setPWM(0,0,angle);
	PCA9685_setPWM(1,0,angle);
	PCA9685_setPWM(2,0,angle);
	PCA9685_setPWM(3,0,angle);
	PCA9685_setPWM(4,0,angle);
	PCA9685_setPWM(5,0,angle);
	PCA9685_setPWM(6,0,angle);
	PCA9685_setPWM(7,0,angle);
	PCA9685_setPWM(8,0,angle);
	PCA9685_setPWM(9,0,angle);
	PCA9685_setPWM(10,0,angle);
	PCA9685_setPWM(11,0,angle);
	PCA9685_setPWM(12,0,angle);
	PCA9685_setPWM(13,0,angle);
	PCA9685_setPWM(14,0,angle);
	PCA9685_setPWM(15,0,angle);

	// 延迟100ms
    usleep(100000);
}

/**********************************
 * 函数全称: 
 * void PCA9685_Write(u8 addr,u8 data)
 * 
 * 参数注解:
 * u8 addr (要向PCA9685写入数据的地址)
 * 
 * u8 data (要向PCA9685写入的一个字节的数据)
 * 
 * 函数作用:
 * 向PCA9685模块中写一个字节的数据
 * ************************************/
void PCA9685_Write(u8 addr,u8 data)
{
	IIC_Start();
	
	IIC_Send_Byte(PCA_Addr);
	IIC_NAck();
	
	IIC_Send_Byte(addr);
	IIC_NAck();
	
	IIC_Send_Byte(data);
	IIC_NAck();
	
	IIC_Stop();
}

/**********************************
 * 函数全称: 
 * u8 PCA9685_Read(u8 addr)
 * 
 * 参数注解:
 * u8 addr (要从PCA9685读出数据的地址)
 * 
 * 函数作用:
 * 从PCA9685模块中读出一个字节的数据
 * ************************************/
u8 PCA9685_Read(u8 addr)
{
	u8 data;
	
	IIC_Start();
	
	IIC_Send_Byte(PCA_Addr);
	IIC_NAck();
	
	IIC_Send_Byte(addr);
	IIC_NAck();
	
	IIC_Stop();
	
	// 延时10us
	usleep(10);  
	
	IIC_Start();

	IIC_Send_Byte(PCA_Addr | 0x01);
	IIC_NAck();
	
	data = IIC_Read_Byte(0);
	
	IIC_Stop();
	
	return data;
}

/**********************************
 * 函数全称: 
 * void PCA9685_setPWM(u8 num,u32 on,u32 angle)
 * 
 * 参数注解:
 * u8 num (要从PCA9685输出出的PWM的通道序号):
 *     0 ~ 15 (PCA9685只有16路PWM输出)
 * 
 * u32 on:
 *     通常设置为0
 * 
 * u32 angle (调节角度):
 *     0 ~ 180
 * 
 * 函数作用:
 * 调节PCA9685的PWM的占空比,使其旋转到相应角度
 * ************************************/
void PCA9685_setPWM(u8 num,u32 on,u32 angle)
{
	// 定义off值,与on值做对比
	u32 off = ((SERVO_MIN_DUTY_CYCLE + ((angle * (SERVO_MAX_DUTY_CYCLE - SERVO_MIN_DUTY_CYCLE)) / SERVO_MAX_DEGREE)) * SERVO_EXTEND_BIT); 

	IIC_Start();
	
	IIC_Send_Byte(PCA_Addr);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(LED0_ON_L + 4 * num);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(on & 0xFF);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(on >> 8);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(off & 0xFF);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(off >> 8);
	IIC_Wait_Ack();
	
	IIC_Stop();
}

/**********************************
 * 函数全称: 
 * void PCA9685_setFreq(float freq)
 * 
 * 参数注解:
 * float freq (要设置的PCA9685的输出频率, 单位是HZ)
 * 
 * 函数作用:
 * 设置PCA9685的输出频率,并根据频率计算PRE_SCALE的值
 * (PCA9685的16路PWM输出频率是一致的,所以是不能实现不同引脚不同频率的)
 * ************************************/
void PCA9685_setFreq(float freq)
{
	u8 prescale, oldmode, newmode;
	
	double prescaleval;
	
	// 校正频率设置中的过冲
	freq *= 0.92;
	prescaleval = 25000000;
	prescaleval /= 4096;
	prescaleval /= freq;
	prescaleval -= 1;
	prescale = floor(prescaleval + 0.5f);  // 向下取整
	oldmode = PCA9685_Read(PCA_Model);
	
	newmode = (oldmode & 0x7F) | 0x10;
	PCA9685_Write(PCA_Model, newmode);
	PCA9685_Write(PCA_Pre, prescale);
	PCA9685_Write(PCA_Model, oldmode);
	// 延迟5ms
    usleep(5000);
	PCA9685_Write(PCA_Model, oldmode | 0xa1);
}

/**********************************
 * 函数全称: 
 * void setAngle(u8 num,u8 angle)
 * 
 * 参数注解:
 * u8 num (要从PCA9685输出出的PWM的通道序号):
 *     0 ~ 15 (PCA9685只有16路PWM输出)
 * 
 * u8 angle (要设置的舵机的角度)
 * 
 * 函数作用:
 * 设置PCA9685对应PWM通道的舵机的角度
 * ************************************/
void setAngle(u8 num,u8 angle)
{
	u32 off = 0;
	off = (u32)(158 + angle * 2.2);
	PCA9685_setPWM(num, 0, off);
}

