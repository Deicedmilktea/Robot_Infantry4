#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#include "math.h"
pid_struct_t supercap_pid;
motor_info_t  motor_can1[5];       //电机信息结构体, 0123为底盘，4为云台
chassis_t chassis_motor[4];
fp32 superpid[3] = {120,0.1,0};
volatile int16_t Vx=0,Vy=0,Wz=0;
int16_t Temp_Vx;
int16_t Temp_Vy;
int fllowflag = 0;
int chassis_mode_flag = 0;
int8_t chassis_choice_flag = 0;
extern RC_ctrl_t rc_ctrl;
extern ins_data_t ins_data;
extern float powerdata[4];
extern uint16_t shift_flag;
int16_t chassis_mode = 1;//判断底盘状态，用于UI编写
int16_t shot_mode = 0;
// fp32 rx=0.2,ry=0.2;
fp32 L = 0.26; //全向轮到中点的距离
fp32 max_speed = 2000;

//speed mapping
int16_t Speedmapping(int value, int from_min, int from_max, int to_min, int to_max){
	  // 首先将输入值从 [a, b] 映射到 [0, 1] 范围内
    double normalized_value = (value*1.0 - from_min) / (from_max - from_min);
    
    // 然后将标准化后的值映射到 [C, D] 范围内
    int16_t mapped_value = (int16_t)(to_min + (to_max - to_min) * normalized_value);
    
    return mapped_value;
}

void Calculate_speed(){
	Vx=Speedmapping(rc_ctrl.rc.ch[2],-660,660,-max_speed,max_speed);// left and right
	Vy=Speedmapping(rc_ctrl.rc.ch[3],-660,660,-max_speed,max_speed);// front and back
	Wz=Speedmapping(rc_ctrl.rc.ch[4],-660,660,-2*max_speed,2*max_speed);// rotate      
}

// void RC_move(){
// 		motor_speed_target[CHAS_LF] =  Vy + Vx + 3*Wz*(rx+ry);
//     motor_speed_target[CHAS_RF] = -Vy + Vx + 3*Wz*(rx+ry);
//     motor_speed_target[CHAS_RB] = -Vy - Vx + 3*Wz*(rx+ry);
//     motor_speed_target[CHAS_LB] =  Vy - Vx + 3*Wz*(rx+ry);
// }
 
void Chassis_task(void const *pvParameters)
{
    Chassis_loop_Init();
    for(;;)
    {   
				// chassis_mode = rc_ctrl.rc.s[0];//1，3，2
				Calculate_speed();
				// if(chassis_mode==1){
				// 		chassis_motor[0].target_speed =  1000;
				// 		chassis_motor[1].target_speed =  0;
				// 		chassis_motor[2].target_speed =  0;
				// 		chassis_motor[3].target_speed =  0;
				// }
				// if(chassis_mode==2){
				// 		RC_move();
				// }

				chassis_motor_speed_calculate();

				chassis_current_give();
        osDelay(1);

    }

}


static void Chassis_loop_Init()
{
	Vx = 0;
	Vy = 0;
	Wz = 0;

  for(int i=0; i<4; i++)
  {
    chassis_motor[i].pid_value[0] = 30;
    chassis_motor[i].pid_value[1] = 0.5;
    chassis_motor[i].pid_value[2] = 10;
  }

  for(int i=0; i<4; i++)
  {
    chassis_motor[i].target_speed = 0;
  }

  for(int i=0; i<4; i++)
  {
    pid_init(&chassis_motor[i].pid, chassis_motor[i].pid_value, 6000, 6000);
  }
}

/*****************************运动解算************************************/
void chassis_motor_speed_calculate()
{
    chassis_motor[0].target_speed =  Vx*cos(pi/4) + Vy*cos(pi/4) + Wz*L;
    chassis_motor[1].target_speed = -Vx*cos(pi/4) + Vy*cos(pi/4) + Wz*L;
    chassis_motor[2].target_speed = -Vx*cos(pi/4) - Vy*cos(pi/4) + Wz*L;
    chassis_motor[3].target_speed =  Vx*cos(pi/4) - Vy*cos(pi/4) + Wz*L;
}

/********************************速度限制函数*******************************/
  void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed)  
{
    uint8_t i=0;
    int16_t max = 0;
    int16_t temp =0;
    int16_t max_speed = limit_speed;
    fp32 rate=0;
    for(i = 0; i<4; i++)
    {
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//求绝对值
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
					{
            motor_speed[i] *= rate;
					}

    }

}
/*********************************电机电流控制**********************************/
void chassis_current_give() 
{    
    for(int i=0 ; i<4; i++)
    {
      motor_can1[i].set_current = pid_calc(&chassis_motor[i].pid, chassis_motor[i].target_speed, motor_can1[i].rotor_speed);
    }
		
    set_motor_current_can1(0, motor_can1[0].set_current, motor_can1[1].set_current, motor_can1[2].set_current, motor_can1[3].set_current);
}






