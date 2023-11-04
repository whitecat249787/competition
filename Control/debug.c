#include "debug.h"
#include "usart.h"
#include "basic_action.h"
#include "dji_motor.h"

#ifndef BASIC_ACTION
#define BASIC_ACTION

#endif

#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart==&huart7){
		if(recv_data=='s'){
			Change_dji_speed(1,1000);
		}
		
		if(recv_data=='m'){
			Begin_action_one();
		}
		HAL_UART_Receive_IT(&huart7, &recv_data, 1);
	}
}
#endif
