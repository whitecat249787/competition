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

/**************�ڲ��궨����������begin**************/
#define diameter 150//�������ֱ��
#define height 100
#define x_extra 185.64//�˴��Ƕ�άƽ���л�еצ�������е��ĩ�˵ľ���
#define y_extra 53.82

/**************�ڲ��궨����������end**************/

/**************�ڲ������뺯��begin**************/

static uint8_t receiveData[0];
extern uint8_t way[1];
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
float ans=0;//���ڽ�������ʱ�ݴ�����
float x_data=0,y_data=0,z_data=0;//���ڽ�������ʱ�ݴ�����
int count_tim2=0;//���ڶ�ʱ��tim2ִ�д����ļ���
int count_tim3=0;//���ڶ�ʱ��tim3ִ�д����ļ���
float x_bucket,y_bucket,z_bucket;//���ڴ洢Ͱ������

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
	/*      ��ʼ��־   ���ID      Ŀ��ת��   �¸���������*/
	/*1*/{		 END,        0,        8192    ,NULL},
	/*2*/{		 END,        1,        8192*2  ,NULL},
	/*3*/{		 END,        5,        8192*3  ,NULL},
	/*1*/{	     END,        0,        0     ,NULL},
	/*2*/{		 END,        1,          0     ,NULL},
	/*3*/{		 END,        5,           0    ,change_next},
};
static int action_one_state_num=sizeof(action_one_s) / sizeof(action_one_s[0]);//״̬����
/**************�ڲ������뺯��end**************/


/**************�ⲿ�ӿ�begin**************/
void Begin_action_one(void);
void Action_one_content(void);
/**************�ⲿ�ӿ�end**************/


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
1.�������ܣ�
2.��Σ�
3.����ֵ��
4.�÷�������Ҫ��
5.������
*/
void Begin_action_one(void){
	action_one_s[0].ifstart=START;
}
/*
1.�������ܣ�
2.��Σ�
3.����ֵ��
4.�÷�������Ҫ��
5.������
*/
#if 0
void Action_one_content(void){
	static int state_cnt=0;
	if(action_one_s[state_cnt].ifstart==START){//��˳��ÿ�������ɼȶ��Ĺ���
		Change_dji_loc(action_one_s[state_cnt].motor_id,action_one_s[state_cnt].target_loc);//���ת���ȶ��ĽǶȻ�����total_angle
		while(state_cnt<=action_one_state_num){
			if(Arrive_motor_target_loc(action_one_s[state_cnt].motor_id,action_one_s[state_cnt].target_loc)){//�жϵ���Ƿ�����ת��һȦ
				action_one_s[state_cnt].ifstart=END;//��Ǵ˴ζ��������
				if(action_one_s[state_cnt].Next_action!=NULL){
					action_one_s[state_cnt].Next_action();//�����νӶ�������תȥ�����νӶ����ĺ���
				}
				if(state_cnt<action_one_state_num-1){
					action_one_s[++state_cnt].ifstart=START;//����һ������׼����ʼ
				}
				else{//����״̬����
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

//���ת�ٵĵ�����pid.c

void Action_one_content(void)//����ִ������1
{
    //�˴����л�е��ĩ�˵ĳ�ʼ���꣬������ó�x���꣬y����Ĭ��Ϊ0����Ĭ�ϻ�е�۳�ʼʱ��x���غ�
    //���λ��Ϊ��е��ĩ�˶��ǻ�еצ����
    count_tim2=0;
    HAL_TIM_Base_Start_IT(&htim2);//ÿ3sִ��һ�ζ�������ִ������
}

void Action_two_content(void)//����ִ������2
{
    float distance=100;//��Ҫ��е��ĩ�˵�צ���ڲ���ߵ�ľ������ڼ����е��ĩ���赽���λ�ã�������Ϊ100mm
    float x,y,z;//�洢��е��ĩ���赽���λ��
    //ץȡ����Ϊ��0��300������
    x=0;
    y=300;
    z=diameter+distance;
    underpan_roll(x,y);
    float x_temp,y_temp;
    x_temp=sqrt(x*x+y*y)-x_extra;//
    y_temp=z+y_extra;//
    route(x_temp,y_temp);
    //���ǵ���еצ���ܻ���ҡ�Σ��˴�Ӧ����������Ƿ����ʱ
    
    x_bucket=0;
    y_bucket=-1*300;
    z_bucket=z;//�õ�Ͱ������
    
    HAL_TIM_Base_Start_IT(&htim3);//��ʼץ���ƶ�������Ĳ���
}

void Action_three_content(void)//����ִ������3
{
    float x,y,z;
    
    //�������x,y,z����
    HAL_UART_Receive_IT(&huart5,receiveData,1);//UART7�����ڽ���ģʽ������
    x=x_data;
    y=y_data;
    z=z_data;
    x_data=0;//�������ݺ󽫴洢�������ݵı�������
    y_data=0;
    z_data=0;
    
    //�ظ�Action_two_content��ץ����Ķ�����������Ͱ���겻��
    float distance=100;//��Ҫ��е��ĩ�˵�צ���ڲ���ߵ�ľ������ڼ����е��ĩ���赽���λ�ã�������Ϊ100mm
    z=diameter+distance;
    underpan_roll(x,y);
    float x_temp,y_temp;
    x_temp=sqrt(x*x+y*y)-x_extra;
    y_temp=z+y_extra;
    route(x_temp,y_temp);
    //���ǵ���еצ���ܻ���ҡ�Σ��˴�Ӧ����������Ƿ����ʱ
    
    x_bucket=0;
    y_bucket=-1*300;
    z_bucket=z;//�õ�Ͱ������
    
    count_tim3=0;//��ʼץ���ƶ�������Ĳ���
}

void Action_four_content(void)//����ִ������4
{
    float x_ball,y_ball,z_ball;//���ڴ洢�������
    //ȫ�ֱ�������ץ���ƶ��ͷ������Ϊ�������ŵ���ʱ��������
    
    //�������Ͱ��x,y,z����
    
    //�������x,y,z����
    HAL_UART_Receive_IT(&huart5,receiveData,1);//UART7�����ڽ���ģʽ������
    x_ball=x_data;
    y_ball=y_data;
    z_ball=z_data;
    x_data=0;//�������ݺ󽫴洢�������ݵı�������
    y_data=0;
    z_data=0;
    
    //����Ͱ��x,y,z����
    HAL_UART_Receive_IT(&huart5,receiveData,1);//UART7�����ڽ���ģʽ������
    x_bucket=x_data;
    y_bucket=y_data;
    z_bucket=z_data;
    x_data=0;//�������ݺ󽫴洢�������ݵı�������
    y_data=0;
    z_data=0;
    
    //�ظ�Action_two_content��ץ����Ķ��������Ͱ��������ı�
    float distance=100;//��Ҫ��е��ĩ�˵�צ���ڲ���ߵ�ľ������ڼ����е��ĩ���赽���λ�ã�������Ϊ100mm
    z_ball=diameter+distance;
    underpan_roll(x_ball,y_ball);
    float x_temp,y_temp;
    x_temp=sqrt(x_ball*x_ball+y_ball*y_ball)-x_extra;
    y_temp=z_ball+y_extra;
    route(x_temp,y_temp);
    
    //Action��ֻ������ƶ������λ�ú͵õ�Ͱ�����겢����x,y,z_bucket�Ĺ���
    
    count_tim3=0;//��ʼץ���ƶ�������Ĳ���
}
//��ע�⣬ʵ��Ӧ�ó���ʱӦע�����ĺ������Գ����ڵ����id�����޸ģ�ͬʱע������ģʽ��Ӧ����Ϊλ��ģʽ��������������������������������

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==UART7)
    {
        uint8_t one='1',two='2',three='3',four='4';
        if(way[0]==one)
            {
                printf("m1\n");
                Action_one_content();//����������Ϊ1����ִ������1
            }
            if(way[0]==two)
            {
                printf("m2\n");
                Action_two_content();//����������Ϊ2����ִ������2
            }
            if(way[0]==three)
            {
                printf("m3\n");
                Action_three_content();//����������Ϊ3����ִ������3
            }
            if(way[0]==four)
            {
                printf("m4\n");
                Action_four_content();//����������Ϊ4����ִ������4
            }
        HAL_UART_Receive_IT(&huart7,way,1);//����������ݶ�λ��ģʽ�����У�ģʽ����λ��basic_action.c;
    }
    if(huart->Instance==UART5)
    {
        //�˴��涨Э�飺����λ��������Ϣʱ����#��ͷ���м���x,y,z�������磺����(100,200,300)ʱ��������"#100x200y300z";
        //�˴�����֪���ֱ����z�����ɲ�ȡ
        
        if(receiveData[0]=='#')
        {
            printf("data receive begin\n");
        }
        //����x����
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
        
        //����y����
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
        
        //����z����
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

//��ʱ������ʹ��е��ĩ����Ŀ��λ��ͣ��һ��ʱ��
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM2)
  {
    count_tim2++;//������һ�У�ʹ���ÿ��3sǰ����һ����
    if(count_tim2==1)
    {
        //��һ����Ϊ(300,0,0),�˴����л�е��ĩ�˵ĳ�ʼ���꣬������ó�x���꣬y����Ĭ��Ϊ0����Ĭ�ϻ�е�۳�ʼʱ��x���غ�
        underpan_roll(300,0);//����Ŀ�������ά����ϵ�е�x,y���꣬�жϵ���ת��
        
        float x1,y1;//�˴�x,yΪ����·��������x,y������е���ڵ�ǰ��άƽ���ڵ�Ŀ��ˮƽ�������ֱ����
        x1=sqrt(300*300+0*0);
        y1=0;
        route(x1,y1);
    }
    if(count_tim2==2)
    {
        //�ڶ�����Ϊ(200,50,150)
        
        underpan_roll(200,50);
    
        float x2,y2;
        x2=sqrt(200*200+50*50);
        y2=150;
        route(x2,y2);
    }
    if(count_tim2==3)
    {
        //��������Ϊ(-150,-150,50)
    
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
      if(count_tim3==2)//��֤ץȡǰ����ͣ��3s~6s
      {
          //ץ��
          grab();
      }
      if(count_tim3==3)//����е�۾����ƶ�ʱ��ȷ����ʱʱ�䳤��
      {
          //�ƶ�
          float x_temp,y_temp;
          z_bucket+=height+100;//Ԥ��100mm����ʹ�������²���Ͱ���Ϸ�100mm��
          underpan_roll(x_bucket,y_bucket);
          x_temp=sqrt(x_bucket*x_bucket+y_bucket*y_bucket)-x_extra;
          y_temp=z_bucket+y_extra;
          route(x_temp,y_temp);
      }
      if(count_tim3==4)
      {
          //����
          grab();
          x_bucket=0;
          y_bucket=0;
          z_bucket=0;//��ɶ�����Ͱ��������0�������´�ʹ��;;
      }
      
  }
}