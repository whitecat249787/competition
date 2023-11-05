#include "main.h"
#include "math.h"
#include "dji_motor.h"
#include "stdio.h"
#define motor_id1 4//电机id应修改为当前使用电机的id！！！！！！！！！！！！！！！！！！
#define pi acos(-1)
//此函数用于将底盘转过相应的角度，使得上方的机械臂能够正对所需到达的点（球）
void underpan_roll(float x,float y);