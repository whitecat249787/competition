#include "calculate.h"
float l1=196;//��е�۳���
float l2=250;
//�ú����贫���е�����赽���x,y���꣬��������е�۵�ת��theta_1[0]��theta_2[0];
void calculate(float theta_1[],float theta_2[])
{
	float alaph,beta;
	float x,y;
	x=theta_1[0];
	y=theta_2[0];
	alaph=atan(y/x)+acos((x*x+y*y+l1*l1-l2*l2)/(2*sqrt(x*x+y*y)*l1));
	beta=atan((y-l1*sin(alaph))/(x-l1*cos(alaph)));
	theta_1[0]=alaph;
	theta_2[0]=beta;
}