#include "basic_action.h"
#include "main.h"
#include "dji_motor.h"
#include "basic.h"
#include "usart.h"
#include "move.h"
#include "underpan.h"
#include "route.h"
#include "grab.h"
#include "stdio.h"

/**************内部宏定义与重命名begin**************/
#define diameter 150//定义球的直径
#define height 100
#define x_extra 185.64//此处是二维平面中机械爪顶端与机械臂末端的距离
#define y_extra 53.82

/**************内部宏定义与重命名end**************/

/**************内部变量与函数begin**************/

static uint8_t receiveData[0];
extern uint8_t way[1];
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
float ans=0;//用于接收数据时暂存数据
float x_data=0,y_data=0,z_data=0;//用于接收数据时暂存数据
int count_tim2=0;//用于定时器tim2执行次数的计数
int count_tim3=0;//用于定时器tim3执行次数的计数
float x_bucket,y_bucket,z_bucket;//用于存储桶的坐标

static uint8_t Arrive_motor_target_loc(int motor_id,int target_angle);
static void change_next(void);
const int equal_allow_err=100;
//int fputc(int c,FILE *stream)
//{
//  uint8_t ch[1]={c};
//  HAL_UART_Transmit(&huart7,ch,1,0XFFFF);
//  return c;
//}
static action_member action_one_s[]={
	/*      开始标志   电机ID      目标转程   下个动作函数*/
	/*1*/{		 END,        0,        8192    ,NULL},
	/*2*/{		 END,        1,        8192*2  ,NULL},
	/*3*/{		 END,        5,        8192*3  ,NULL},
	/*1*/{	     END,        0,        0     ,NULL},
	/*2*/{		 END,        1,          0     ,NULL},
	/*3*/{		 END,        5,           0    ,change_next},
};
static int action_one_state_num=sizeof(action_one_s) / sizeof(action_one_s[0]);//状态总数
/**************内部变量与函数end**************/


/**************外部接口begin**************/
void Begin_action_one(void);
void Action_one_content(void);
/**************外部接口end**************/


static uint8_t Arrive_motor_target_loc(int motor_id,int target_angle){
	if (Basic_int_abs(Get_dji_information(motor_id).total_angle-target_angle)<equal_allow_err){
		return 1;
	}
	else{
		return 0;
	}
}

static void change_next(void){
	USART_printf("over\n");
}

/*
1.函数功能：
2.入参：
3.返回值：
4.用法及调用要求：
5.其它：
*/
void Begin_action_one(void){
	action_one_s[0].ifstart=START;
}
/*
1.函数功能：
2.入参：
3.返回值：
4.用法及调用要求：
5.其它：
*/
#if 0
void Action_one_content(void){
	static int state_cnt=0;
	if(action_one_s[state_cnt].ifstart==START){//按顺序每个电机完成既定的工作
		Change_dji_loc(action_one_s[state_cnt].motor_id,action_one_s[state_cnt].target_loc);//电机转过既定的角度或重置total_angle
		while(state_cnt<=action_one_state_num){
			if(Arrive_motor_target_loc(action_one_s[state_cnt].motor_id,action_one_s[state_cnt].target_loc)){//判断电机是否完整转完一圈
				action_one_s[state_cnt].ifstart=END;//标记此次动作已完成
				if(action_one_s[state_cnt].Next_action!=NULL){
					action_one_s[state_cnt].Next_action();//若有衔接动作，则转去运行衔接动作的函数
				}
				if(state_cnt<action_one_state_num-1){
					action_one_s[++state_cnt].ifstart=START;//令下一个动作准备开始
				}
				else{//所有状态结束
					state_cnt=0;
				}
			}
			else{
				break;
			}
		}
	}
}
#endif

//电机转速的调节在pid.c

void Action_one_content(void)//程序执行任务1
{
    //此处需有机械臂末端的初始坐标，需测量得出x坐标，y坐标默认为0，即默认机械臂初始时与x轴重合
    //打点位置为机械臂末端而非机械爪顶部
    count_tim2=0;
    HAL_TIM_Base_Start_IT(&htim2);//每3s执行一次动作，共执行三次
}

void Action_two_content(void)//程序执行任务2
{
    float distance=100;//需要机械臂末端到爪子内侧最高点的距离用于计算机械臂末端需到达的位置，暂且设为100mm
    float x,y,z;//存储机械臂末端需到达的位置
    //抓取坐标为（0，300）的球
    x=0;
    y=300;
    z=diameter+distance;
    underpan_roll(x,y);
    float x_temp,y_temp;
    x_temp=sqrt(x*x+y*y)-x_extra;//
    y_temp=z+y_extra;//
    route(x_temp,y_temp);
    //考虑到机械爪可能会有摇晃，此处应视情况决定是否加延时
    
    x_bucket=0;
    y_bucket=-1*300;
    z_bucket=z;//得到桶的坐标
    
    HAL_TIM_Base_Start_IT(&htim3);//开始抓球、移动、放球的步骤
}

void Action_three_content(void)//程序执行任务3
{
    float x,y,z;
    
    //接收球的x,y,z坐标
    HAL_UART_Receive_IT(&huart5,receiveData,1);//UART7已用于接收模式的类型
    x=x_data;
    y=y_data;
    z=z_data;
    x_data=0;//接收数据后将存储接收数据的变量清零
    y_data=0;
    z_data=0;
    
    //重复Action_two_content中抓放球的动作，其中球桶坐标不变
    float distance=100;//需要机械臂末端到爪子内侧最高点的距离用于计算机械臂末端需到达的位置，暂且设为100mm
    z=diameter+distance;
    underpan_roll(x,y);
    float x_temp,y_temp;
    x_temp=sqrt(x*x+y*y)-x_extra;
    y_temp=z+y_extra;
    route(x_temp,y_temp);
    //考虑到机械爪可能会有摇晃，此处应视情况决定是否加延时
    
    x_bucket=0;
    y_bucket=-1*300;
    z_bucket=z;//得到桶的坐标
    
    count_tim3=0;//开始抓球、移动、放球的步骤
}

void Action_four_content(void)//程序执行任务4
{
    float x_ball,y_ball,z_ball;//用于存储球的坐标
    //全局变量，将抓球、移动和放球抽象为函数，放到定时器函数中
    
    //接收球和桶的x,y,z坐标
    
    //接收球的x,y,z坐标
    HAL_UART_Receive_IT(&huart5,receiveData,1);//UART7已用于接收模式的类型
    x_ball=x_data;
    y_ball=y_data;
    z_ball=z_data;
    x_data=0;//接收数据后将存储接收数据的变量清零
    y_data=0;
    z_data=0;
    
    //接收桶的x,y,z坐标
    HAL_UART_Receive_IT(&huart5,receiveData,1);//UART7已用于接收模式的类型
    x_bucket=x_data;
    y_bucket=y_data;
    z_bucket=z_data;
    x_data=0;//接收数据后将存储接收数据的变量清零
    y_data=0;
    z_data=0;
    
    //重复Action_two_content中抓放球的动作，球和桶的坐标均改变
    float distance=100;//需要机械臂末端到爪子内侧最高点的距离用于计算机械臂末端需到达的位置，暂且设为100mm
    z_ball=diameter+distance;
    underpan_roll(x_ball,y_ball);
    float x_temp,y_temp;
    x_temp=sqrt(x_ball*x_ball+y_ball*y_ball)-x_extra;
    y_temp=z_ball+y_extra;
    route(x_temp,y_temp);
    
    //Action中只需完成移动到球的位置和得到桶的坐标并传入x,y,z_bucket的工作
    
    count_tim3=0;//开始抓球、移动、放球的步骤
}
//需注意，实际应用程序时应注意电机的号数并对程序内电机的id进行修改，同时注意电机的模式，应该设为位置模式！！！！！！！！！！！！！！！！

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==UART7)
    {
        uint8_t one='1',two='2',three='3',four='4';
        if(way[0]==one)
            {
                printf("m1\n");
                Action_one_content();//若串口输入为1，则执行任务1
            }
            if(way[0]==two)
            {
                printf("m2\n");
                Action_two_content();//若串口输入为2，则执行任务2
            }
            if(way[0]==three)
            {
                printf("m3\n");
                Action_three_content();//若串口输入为3，则执行任务3
            }
            if(way[0]==four)
            {
                printf("m4\n");
                Action_four_content();//若串口输入为4，则执行任务4
            }
        HAL_UART_Receive_IT(&huart7,way,1);//坐标的输入暂定位于模式函数中，模式函数位于basic_action.c;
    }
    if(huart->Instance==UART5)
    {
        //此处规定协议：输入位置坐标信息时，由#开头，中间由x,y,z隔开，如：输入(100,200,300)时，请输入"#100x200y300z";
        //此处因已知球的直径，z坐标或可不取
        
        if(receiveData[0]=='#')
        {
            printf("data receive begin\n");
        }
        //接收x数据
        if(receiveData[0]=='x')
        {
            x_data=ans;
            ans=0;
        }
        else
        {
            int temp;
            temp=receiveData[0]-'0';
            if(temp>=0&&temp<=9)
            {
                ans=ans*10;
                ans+=temp; 
            }
        }
        
        //接收y数据
        if(receiveData[0]=='y')
        {
            y_data=ans;
            ans=0;
        }
        else
        {
            int temp;
            temp=receiveData[0]-'0';
            if(temp>=0&&temp<=9)
            {
                ans=ans*10;
                ans+=temp; 
            }
        }
        
        //接收z数据
        if(receiveData[0]=='z')
        {
            z_data=ans;
            ans=0;
        }
        else
        {
            int temp;
            temp=receiveData[0]-'0';
            if(temp>=0&&temp<=9)
            {
                ans=ans*10;
                ans+=temp; 
            }
        }
        
        HAL_UART_Receive_IT(&huart5,receiveData,1);
    }
}

//定时器用于使机械臂末端在目标位置停留一段时间
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM2)
  {
    count_tim2++;//在任务一中，使电机每隔3s前往下一个点
    if(count_tim2==1)
    {
        //第一个点为(300,0,0),此处需有机械臂末端的初始坐标，需测量得出x坐标，y坐标默认为0，即默认机械臂初始时与x轴重合
        underpan_roll(300,0);//传入目标点在三维坐标系中的x,y坐标，判断底盘转角
        
        float x1,y1;//此处x,y为传进路径函数的x,y，即机械臂在当前二维平面内的目标水平坐标和竖直坐标
        x1=sqrt(300*300+0*0);
        y1=0;
        route(x1,y1);
    }
    if(count_tim2==2)
    {
        //第二个点为(200,50,150)
        
        underpan_roll(200,50);
    
        float x2,y2;
        x2=sqrt(200*200+50*50);
        y2=150;
        route(x2,y2);
    }
    if(count_tim2==3)
    {
        //第三个点为(-150,-150,50)
    
        underpan_roll(-150,-150);
        
        float x3,y3;
        x3=sqrt(150*150+150*150);
        y3=50;
        route(x3,y3);
    }
        
  }
  if(htim->Instance==TIM3)
  {
      count_tim3++;
      if(count_tim3==2)//保证抓取前至少停留3s~6s
      {
          //抓球
          grab();
      }
      if(count_tim3==3)//看机械臂具体移动时间确定延时时间长度
      {
          //移动
          float x_temp,y_temp;
          z_bucket+=height+100;//预留100mm的误差，使最后球的下部在桶的上方100mm处
          underpan_roll(x_bucket,y_bucket);
          x_temp=sqrt(x_bucket*x_bucket+y_bucket*y_bucket)-x_extra;
          y_temp=z_bucket+y_extra;
          route(x_temp,y_temp);
      }
      if(count_tim3==4)
      {
          //放球
          grab();
          x_bucket=0;
          y_bucket=0;
          z_bucket=0;//完成动作后将桶的坐标置0，方便下次使用;;
      }
      
  }
}