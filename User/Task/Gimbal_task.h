#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "struct_typedef.h"
#include "pid.h"
#include "Chassis_task.h"
#include "main.h"

#define pi 3.1415926

typedef struct
{
    motor_info_t motor_info;     // 电机信息结构体
    pid_struct_t pid_angle;      // 云台电机angle的pid结构体
    pid_struct_t pid_speed;      // 云台电机speed的pid结构体
    fp32 pid_angle_value[3];     // 云台电机angle的pid参数
    fp32 pid_speed_value[3];     // 云台电机speed的pid参数
    fp32 target_angle;           // 云台电机的目标角度
    fp32 target_speed;           // 云台电机的目标速度
    fp32 init_angle;             // 云台电机的初始角度
} gimbal_t;

void Gimbal_task(void const *pvParameters);


void Gimbal_loop_init();

// /*将目标角度从（-pi, pi）映射到（0, 8091）*/
// float angle_map(float cur_angle);

/*角度过零处理*/
void angle_over_zero(float err);

/*接收陀螺仪数据转化给电机*/
void gyro_receive();

/*接收遥控器数据控制云台旋转*/
void remote_gimbal_control();

/*角度范围限制*/
void detel_calc(fp32 *angle);

/*can1发送电流*/
void gimbal_can1_cmd(int16_t v3);

#endif