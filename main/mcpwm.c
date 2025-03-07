#include"mcpwm.h"

/**********************************
 * 函数全称: 
 * void InitServo()
 * 
 * 函数作用:
 * 初始化驱动电机的PWM
 * ************************************/
void InitServo()
{
    /**********************************
     * 函数全称: 
     * esp_err_t mcpwm mcpwm_gpio_init(mcpwm_unit_t mcpwm_num, mcpwm_io_signals_t io_signal, int gpio_num);
     * 
     * 参数注解:
     * mcpwm_unit_t mcpwm_num:
     *     MCPWM_UNIT_0 / MCPWM_UNIT_1 (ESP32上有两个MCPWM单元)
     * 
	 * mcpwm_io_signals_t io_signal：
	 *     MCPWM0A / MCPWM0B / MCPWM1A / MCPWM1B / MCPWM2A / MCPWM2B: pwm输出信号
	 * 	   MCPWM_SYNC_0 / MCPWM_SYNC_1 / MCPWM_SYNC_2：同步输入信号
	 * 	   MCPWM_FAULT_0 / MCPWM_FAULT_1 / MCPWM_FAULT_2：故障输入信号
	 * 	   MCPWM_CAP_0 / MCPWM_CAP_1 / MCPWM_CAP_2：捕获输入信号
     * 
     * int gpio_num:
     *     ESP32上的GPIO号 (注: GPIO34到39不能产生PWM)
     * 
     * 函数作用:
     * 初始化mcpwm所使用的到的引脚
	 * ************************************/
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_GPIO1);    
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, SERVO_GPIO2);    
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, SERVO_GPIO3); 
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, SERVO_GPIO4);
	
    // 定义初始化mcpwm配置的结构体
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;  // 输出mcpwm波形频率 Hz (frequency = 50Hz, i.e. for every servo motor time period should be 20ms)
    pwm_config.cmpr_a = 0;  // 操作器A输出占空比为0 (duty cycle of PWMxA = 0)
    pwm_config.cmpr_b = 0;  // 操作器B输出占空比为0 (duty cycle of PWMxb = 0)
    /*******************************************
     *duty_mode (占空比类型)：
     *	  MCPWM_DUTY_MODE_0： 高有效，例如：%20占空比 == 高电平20%
     *	  MCPWM_DUTY_MODE_1： 低有效，例如：%20占空比 == 低电平20%
     *	  MCPWM_HAL_GENERATOR_MODE_FORCE_LOW： 输出强制为低
     *	  MCPWM_HAL_GENERATOR_MODE_FORCE_HIGH：输出强制为高
     ********************************************/
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;  // 设置占空比为高电平有效
    /****************************************
     * counter_mode (计数模式):
     * 	  MCPWM_FREEZE_COUNTER: 计数器冻结
     * 	  MCPWM_UP_COUNTER:  计数器递增,左对齐，频率不变
     * 	  MCPWM_DOWN_COUNTER: 计数器递减,右对齐，频率不变
     * 	  MCPWM_UP_DOWN_COUNTER：计数器递增递减，中间对齐，频率减半
     ********************************************/
    pwm_config.counter_mode = MCPWM_UP_COUNTER;  //设置计数器递增

    /**********************************
     * 函数全称: 
     * esp_err_t mcpwm_init(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, const mcpwm_config_t *mcpwm_conf)
     * 
     * 参数注解:
     * mcpwm_unit_t mcpwm_num (组号):
     *     MCPWM_UNIT_0 / MCPWM_UNIT_1 (ESP32上有两个MCPWM单元)
     * 
	 * mcpwm_timer_t timer_num (定时器号)：
	 *     MCPWM_TIMER_0 / MCPWM_TIMER_1 / MCPWM_TIMER_2 :定时器选定 (ESP32的一个mcpwm单元有3个定时器)
     * 
     * const mcpwm_config_t *mcpwm_conf:
     *     ESP32的mcpwm配置
     * 
     * 函数作用:
     *     初始化并启动一组mcpwm
	 * ************************************/
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  // 使用上述的配置对0号mcpwm单元的PWM0A和PWM0B进行配置 (Configure PWM0A & PWM0B with above settings)
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);  // 使用上述的配置对1号mcpwm单元的PWM0A和PWM0B进行配置 (Configure PWM0A & PWM0B with above settings)
}

/**********************************
 * 函数全称: 
 * static float servo_per_degree_init(uint32_t degree_of_rotation)
 * 
 * 参数注解:
 * uint32_t degree_of_rotation (伺服电机角度):
 *     0 ~ 180 (当前所用的伺服电机的角度只支持0到180度)
 * 
 * 函数作用:
 * 将伺服电机角度转化成脉冲宽度
 * ************************************/
float servo_per_degree_init(uint32_t degree_of_rotation)
{
    // 定义计算出的脉冲宽度变量
    float cal_pulsewidth = 0;
    // 角度转us脉冲宽度计算公式
    cal_pulsewidth = (float)(SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    // 返回计算出的脉冲宽度
    return cal_pulsewidth;   
}

/**********************************
 * 函数全称: 
 * void SetServoAngle(int adress, char mode, int angle)
 * 
 * 参数注解:
 * int adress (使用的MCPWM单元号):
 *     0 / 1 (ESP32上有两个MCPWM单元)
 * 
 * char mode (使用的MCPWM单元上的PWM输出通道)：
 *     A / B (ESP32的一个定时器和操作器对应PWMA和PWMB)
 * 
 * int angle (要设置的伺服电机旋转的角度):
 *     0 ~ 180 (当前所用的伺服电机的角度只支持0到180度)
 * 
 * 函数作用:
 * 设置伺服电机旋转角度
 * ************************************/
void SetServoAngle(int adress, char mode, int angle)
{
    // 定义角度转脉冲变量并对输入的角度进行脉冲宽度转换
    float angle_pulse = servo_per_degree_init(angle);
    
    // 对当前要调节的PWM通道进行筛选，然后进行脉冲宽度调节
    /**********************************
     * 函数全称: 
     * esp_err_t mcpwm_set_duty_in_us(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, mcpwm_generator_t gen, float duty)
     * 
     * 参数注解:
     * mcpwm_unit_t mcpwm_num (组号):
     *     MCPWM_UNIT_0 / MCPWM_UNIT_1 (ESP32上有两个MCPWM单元)
     * 
     * mcpwm_timer_t timer_num (定时器号)：
	 *     MCPWM_TIMER_0 / MCPWM_TIMER_1 / MCPWM_TIMER_2 :定时器选定 (ESP32的一个mcpwm单元有3个定时器)
     * 
     * mcpwm_generator_t gen (使用的MCPWM单元上的PWM输出通道)：
     *     A / B (ESP32的一个定时器和操作器对应PWMA和PWMB)
     * 
     * float duty (表示占空比百分数%[或微秒]):
     *     500 ~ 2500 (当前所用的伺服电机的角度只支持0到180度,因而以微秒为单位，处于500us ~ 2500us)
     * 
     * 函数作用：
     * 设置PWM占空比
     * ************************************/
    if(adress == 0 && mode == 'A')
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle_pulse);
    else if(adress == 0 && mode == 'B')
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle_pulse);
    else if(adress == 1 && mode == 'A')
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, angle_pulse);
    else if(adress == 1 && mode == 'B')
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, angle_pulse);
}