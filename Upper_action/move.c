#include "move.h"
float k1=30;//减速比
float k2=30;
void move(float x,float y)
{
    float theta1_begin;
    float theta0_begin;
    theta0_begin=pi/6;
    theta1_begin=pi/3;
  float theta1_0;//机械臂初始角度，此处需改动，应改为当前机械臂的角度，已适应已经转动过的情况，可能可以直接获取total_angle来得到
  float theta0_0;
  motor_measure_t message;
  message=Get_dji_information(motor_id2);
  theta0_0=message.total_angle/8192*2*pi/k1+theta0_begin;
  message=Get_dji_information(motor_id3);
  theta1_0=message.total_angle/8192*2*pi/k2+theta0_begin;
  //此处获得两个电机的转角，测得减速比后可以算出机械臂的转角
  float theta1[1];
  float theta2[1];
  theta1[0]=x;
  theta2[0]=y;
  calculate(theta1,theta2);
  float deta_theta1=theta1[0]-theta1_0;
  float deta_theta2=theta2[0]-theta0_0;
  float Targettheta1=deta_theta1*k1;
  float Targettheta2=deta_theta2*k2;
  float Target_loc1=Targettheta1/(2*pi)*8192;
  float Target_loc2=Targettheta1/(2*pi)*8192;
  Change_dji_loc(motor_id2,Target_loc1);
  Change_dji_loc(motor_id3,Target_loc2);
}