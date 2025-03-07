#include "robot_state.h"

/**********************************
 * 函数全称: 
 * void Robot_Walk(double t, double *angles)
 * 
 * 参数注解:
 * double t (机器人运动中的当前时间)
 * 
 * double *angles (每个自由度(舵机)要设置的角度)
 * 
 * 函数作用:
 * 让机器人直走
 * ************************************/
void Robot_Walk(double t, double *angles)
{
    // 定义机器人的目标运动状态
    struct Phase phases[] = {
        // 改进第六版（行走可使用标准版）
        {{95, 90, 90, 90, 90, 85, 90, 90, 90, 90}, 0.5},  // 立正状态
        {{90, 90, 90, 90, 107, 90, 90, 90, 90, 107}, 0.5},  // 迈右腿，转移重心
        {{90, 90, 90, 90, 107, 85, 55, 90, 113, 96}, 0.5},  // 迈右腿
        {{90, 73, 90, 110, 97, 85, 65, 90, 106, 90}, 0.5},  // 落右脚，后蹬左腿
        {{90, 73, 90, 110, 78, 90, 72, 90, 102, 78}, 0.5},  // 转移重心至右腿，左腿抬起脚尖
        {{90, 73, 90, 120, 78, 90, 90, 90, 90, 78}, 1.0},  // 整体完全转移重心到右脚
        {{94, 90, 50, 60, 78, 90, 86, 90, 90, 78}, 0.5},  // 收左腿
        {{90, 90, 90, 90, 90, 90, 90, 90, 90, 90}, 0.5},  // 立正状态
        {{90, 90, 90, 90, 78, 90, 90, 90, 90, 78}, 0.5},  // 迈左腿，转移重心
        {{95, 107, 90, 81, 78, 90, 90, 90, 90, 78}, 0.5},  // 迈左腿
        {{95, 90, 90, 90, 90, 85, 90, 90, 90, 90}, 0.25},  // 立正状态
    };

    // 计算当前结构体数组长度
    int n = sizeof(phases) / sizeof(struct Phase);

    // 计算机器人当前时间段所应处于的角度和运动状态
    Compute_Angles(phases, n, t, angles, Cycle_time_Walk);

    // printf("第%d段时间步长的电机运动角度是: ", i + 1); 

    // 将角度输出到要设置电机的PWM中 
    Set_Angles_To_PWM(angles);
}

/**********************************
 * 函数全称: 
 * void Robot_Turn_Right(double t, double *angles)
 * 
 * 参数注解:
 * double t (机器人运动中的当前时间)
 * 
 * double *angles (每个自由度(舵机)要设置的角度)
 * 
 * 函数作用:
 * 让机器人右转
 * ************************************/
void Robot_Turn_Right(double t, double *angles)
{
    // 定义机器人的目标运动状态
    struct Phase phases[] = {
        // 改进第八版（右转）
        {{90, 90, 90, 90, 90, 90, 90, 90, 90, 90}, 0.5},  // 立正状态
        {{90, 90, 90, 90, 80, 90, 90, 90, 90, 80}, 0.5},  // 迈左腿，转移重心
        {{95, 105, 90, 81, 82, 90, 90, 90, 90, 80}, 0.5},  // 迈左腿
        {{90, 90, 90, 90, 90, 90, 90, 90, 90, 90}, 0.5}  // 立正状态
    };

    // 计算当前结构体数组长度
    int n = sizeof(phases) / sizeof(struct Phase);

    // 计算机器人当前时间段所应处于的角度和运动状态
    Compute_Angles(phases, n, t, angles, Cycle_time_Turn);

    // printf("第%d段时间步长的电机运动角度是: ", i + 1); 

    // 将角度输出到要设置电机的PWM中 
    Set_Angles_To_PWM(angles);
}

/**********************************
 * 函数全称: 
 * void Robot_Turn_Left(double t, double *angles)
 * 
 * 参数注解:
 * double t (机器人运动中的当前时间)
 * 
 * double *angles (每个自由度(舵机)要设置的角度)
 * 
 * 函数作用:
 * 让机器人左转
 * ************************************/
void Robot_Turn_Left(double t, double *angles)
{
    // 定义机器人的目标运动状态
    struct Phase phases[] = {
        // 改进第七版（左转）
        {{90, 90, 90, 90, 90, 90, 90, 90, 90, 90}, 0.5},  // 立正状态
        {{90, 90, 90, 90, 107, 90, 90, 90, 90, 107}, 0.5},  // 迈右腿，转移重心
        {{90, 90, 90, 90, 107, 85, 83, 90, 94, 96}, 0.5},  // 迈右腿
        {{90, 90, 90, 90, 90, 90, 90, 90, 90, 90}, 0.5}  // 立正状态
    };

    // 计算当前结构体数组长度
    int n = sizeof(phases) / sizeof(struct Phase);

    // 计算机器人当前时间段所应处于的角度和运动状态
    Compute_Angles(phases, n, t, angles, Cycle_time_Turn);

    // printf("第%d段时间步长的电机运动角度是: ", i + 1); 

    // 将角度输出到要设置电机的PWM中 
    Set_Angles_To_PWM(angles);
}

/**********************************
 * 函数全称: 
 * void Compute_Angles(struct Phase *phases, int n, double t, double *angles, double Cycle_time)
 * 
 * 参数注解:
 * struct Phase *phases (目标运动状态的结构体数组)
 * 
 * double t (机器人运动中的当前时间)
 * 
 * double *angles (每个自由度(舵机)要设置的角度)
 * 
 * Cycle_time (当前运动状态的运动周期)
 * 
 * 函数作用:
 * 计算机器人当前角度和运动状态
 * ************************************/
void Compute_Angles(struct Phase *phases, int n, double t, double *angles, double Cycle_time)
{
    // 定义临时处理变量
    int i, temp;

    // 循环计算时间机器人在本运动周期中已经使用的时间
    double phase_t = fmod(t, Cycle_time); 

    printf("此段时间步长中的n: %d, t: %lf, phase_t: %lf, phases[n - 1].duration: %lf \n", n, t, phase_t, phases[n - 1].duration);

    // 找到当前阶段
    for (i = 0; i < n - 1; i++) {
        if (phase_t < phases[i].duration) 
        {
            break;
        }

        phase_t -= phases[i].duration;
    }

    printf("现在处于步态的第%d阶段。",i + 1); 

    // 定义计算角度的系数
    double p = phase_t / phases[i].duration;
    double p1 = 1.0 - p;

    printf("此段时间步长中的p: %lf, p1: %lf\n", p, p1); 
    
    // 计算角度
    for (int j = 0; j < LEG_DOF; j++) {
        // 进行步态循环处理
        if(i + 1 == n)
    	{
    		temp = 0;
		}
		else
		{
			temp = i + 1; 
		}

        // 得到当前舵机应处于角度
        angles[j] = phases[i].angles[j] * p1 + phases[temp].angles[j] * p;
    }
}


/**********************************
 * 函数全称: 
 * void Set_Angles_To_PWM(double *angles)
 * 
 * 参数注解:
 * double *angles (每个自由度（舵机）要设置的角度)
 * 
 * 函数作用:
 * 将角度输出到要设置电机的PWM中 
 * ************************************/
void Set_Angles_To_PWM(double *angles)
{
    // 定义一个1、2、……、10电机对应的PWM通道序号映射的数组
    int PWM_MAP[LEG_DOF] = {15, 14, 13, 12, 11, 0, 1, 2, 3, 4};

    for (int i = 0; i < LEG_DOF; i++) {
        // 通过舵机扩展板设置PWM来设置舵机角度
        PCA9685_setPWM(PWM_MAP[i], 0, angles[i]);
        // 输出角度
        printf("%lf ", angles[i]);
    }

    //控制输出格式
    printf("\n \n");
}