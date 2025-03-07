#ifndef __ROBOT_STATE_H__
#define __ROBOT_STATE_H__

#include <math.h>
#include "PCA9685.h"

// 宏定义机器人运动所需要的相关参数
#define LEG_DOF 10  // 机器人腿部自由度数
#define PI 3.1415926  // 定义π的数值
#define Cycle_time_Walk 5.75  // 定义5.75s的直走运动周期 
#define Cycle_time_Turn 2.0  // 定义2.0s的转向运动周期

// 定义控制机器人的运动状态
struct Phase 
{
    double angles[LEG_DOF];  // 定义机器人十个自由度的运动角度
    double duration;  // 定义当前状态下一个状态的持续时间
};

// 函数声明
void Robot_Walk(double t, double *angles);
void Robot_Turn_Right(double t, double *angles);
void Robot_Turn_Left(double t, double *angles);
void Compute_Angles(struct Phase *phases, int n, double t, double *angles, double Cycle_time);
void Set_Angles_To_PWM(double *angles);

#endif