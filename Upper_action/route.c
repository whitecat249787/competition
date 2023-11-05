#include "route.h"
//该函数需有机械臂末端当前坐标(x0,y0)
//该函数需输入机械臂末端目标坐标(xt,yt)
#define x_begin 314.5//此处定义机械臂末端点初始x坐标，需进行实际测量得出，除臂长外，还需要初始角度，此处非原坐标的x，而是在机械臂的二维平面内的x坐标，此处默认角度为60°，30°
#define y_begin 44.736//此处定义机械臂末端点初始y坐标，需进行实际测量得出，除臂长外，还需要初始角度，此处非原坐标的y，而是原坐标的z，此处默认角度为60°，30°，还需要根据摄像机高度微调，当先从第一个电机处开始计数，之后需加上云台高度
float last_x[50];
float last_y[50];//用于存储上一次操作后的机械臂位置，使得此函数不需要传入当前坐标，此处设置最多可以动50次，可以视情况改动

int cnt=0;//用于计数改变机械臂位置的次数，用于取机械臂的当前位置
void route(float xt,float yt)
{
    if(cnt==0)
    {
        last_x[cnt]=x_begin;
        last_y[cnt]=y_begin;
    }
    
    float x1,y1;//贝塞尔曲线的构造需三个点，此处变量用于存储控制点的坐标
    
    //接下来开始构造第三个点（控制点）
    float x0,y0;//用于存储当前点(机械臂末端起点)的坐标
    x0=last_x[cnt];
    y0=last_y[cnt];
    float d=sqrt((xt-x0)*(xt-x0)+(yt-y0)*(yt-y0));
    x1=xt;
    y1=yt+d;//此处令控制点在终点的正上方，尽量保证抓取爪从球的正上方落下
    
    //接下来取贝塞尔点
    float x_point[100],y_point[100];//该数组用于存储曲线上各点的坐标
    int t0=100;//t0为所取贝塞尔点的个数，具体数值待定，但需注意上方数组的大小需一并改动
    for(int t=1;t<=t0;t++)
    {
        float t_temp=t/t0;//保证0<t_temp<1;
        x_point[t]=pow(1-t_temp,2)*x0+2*t_temp*(1-t_temp)*x1+pow(t_temp,2)*xt;
        y_point[t]=pow(1-t_temp,2)*y0+2*t_temp*(1-t_temp)*y1+pow(t_temp,2)*yt;
        move(x_point[t],y_point[t]);
    }
    cnt++;//机械臂转动次数＋1;
    last_x[cnt]=xt;
    last_y[cnt]=yt;//存储当前坐标进数组
}