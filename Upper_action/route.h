#include "stdio.h"
#include "move.h"
#include "math.h"

//该函数需有机械臂末端当前坐标(x0,y0)
//该函数需输入机械臂末端目标坐标(xt,yt)
//该函数在机械臂末端当前坐标到机械臂末端目标坐标之间构造出贝塞尔曲线
//该函数使机械臂末端能够沿着这条曲线平滑地移动至目标点
//此处默认机械臂正对目标点
void route(float xt,float yt);//已在本函数内调用move函数,可直接用本函数进行电机转动