#include "main.h"
#include "math.h"
#include "dji_motor.h"
#include "stdio.h"
#define motor_id1 4//���idӦ�޸�Ϊ��ǰʹ�õ����id������������������������������������
#define pi acos(-1)
//�˺������ڽ�����ת����Ӧ�ĽǶȣ�ʹ���Ϸ��Ļ�е���ܹ��������赽��ĵ㣨��
void underpan_roll(float x,float y);