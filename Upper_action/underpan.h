#include "main.h"
#include "math.h"
#include "dji_motor.h"
#include "stdio.h"
#define motor_id1 4//Ӧ��Ϊ0
#define pi acos(-1)
//�˺������ڽ�����ת����Ӧ�ĽǶȣ�ʹ���Ϸ��Ļ�е���ܹ��������赽��ĵ㣨��
void underpan_roll(float x,float y);