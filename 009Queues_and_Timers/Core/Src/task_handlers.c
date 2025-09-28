/*
 * task_handlers.c
 *
 *  Created on: Sep 26, 2025
 *      Author: csmla
 */

#include "main.h"

static void process_command (command_t *pcmd_t);
static int extract_command(command_t *pcmd_t);

const char *msg_invalid = "\\\\\\\\\\  Invalid option \\\\\\\n";

/********************************************************************************************/
void menuTask_Handler(void *param){
	uint32_t cmd_addr;
	command_t *pcmd_t;
	uint8_t option;
	const char *msg_menu = "=====================\r\n"
							"|     Menu       |\r\n"
							"=====================\r\n"
							"LedEffect     ------> 0 \r\n"
							"Date and Time ------> 1 \r\n"
							"Exit          ------> 2 \r\n"
							"Enter your choice:      ";
	while(1){
		/* print the menu */
		xQueueSend(printQue_Handle, &msg_menu, 0);

		/* wait until the user option notify from commandTask*/
		xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);

		pcmd_t = (command_t *) cmd_addr;

		if(pcmd_t->len == 1){
			/* ascii to number */
			option = pcmd_t->payload[0] - 48;
			switch(option){
			case 0:
				current_task_state = sLedEffect;
				xTaskNotify(led_Handle, 0, eNoAction);
			break;

			case 1:
				current_task_state = sRtcMenu;
				xTaskNotify(led_Handle, 0, eNoAction);
			break;

			case 2:
			break;

			default:
				xQueueSend(printQue_Handle, &msg_invalid, 0);
			continue;

			}
		}else{
			/* print option is invalid */
			xQueueSend(printQue_Handle, &msg_invalid, 0);
		}

		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

	}

}

/****************************************************************************************/
void ledTask_Handler(void *param){
	uint32_t cmd_addr;
	command_t *pcmd_t;
	const char* msg_led = "========================\r\n"
							  "|      LED Effect     |\r\n"
							  "========================\r\n"
							  "(none,e1,e2,e3,e4)      \r\n"
							  "Enter your choice here : ";
	while(1){

		/*Wait for notification (Notify wait) */
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

		/* print the menu */
		xQueueSend(printQue_Handle, &msg_led, 0);

		/* wait until the user option notify from commandTask*/
		xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);

		pcmd_t = (command_t *) cmd_addr;

		if(pcmd_t->len <= 4){
			if(! strcmp((char*)pcmd_t->payload,"none"))led_effect_stop();
			else if (! strcmp((char*)pcmd_t->payload,"e1"))led_effect(1);
			else if (! strcmp((char*)pcmd_t->payload,"e2"))led_effect(2);
			else if (! strcmp((char*)pcmd_t->payload,"e3"))led_effect(3);
			else if (! strcmp((char*)pcmd_t->payload,"e4"))led_effect(4);
			else{
				/*Tprint invalid message */
				xQueueSend(printQue_Handle, &msg_invalid, 0);
			}

		}else{
			/* print option is invalid */
			xQueueSend(printQue_Handle, &msg_invalid, 0);

			/*update state variable */
			current_task_state = sMainMenu;

			/* Notify menu task */
			xTaskNotify(menu_Handle,0,eNoAction);
		}

		}

}


/********************************************************************************************/
void rtcTask_Handler(void *param){
	while(1){

		/*Wait for notification (Notify wait) */
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

		}
}

/**********************************************************************************************/
void printTask_Handler(void *param){

	char *msg_print;
	while(1){

			xQueueReceive(printQue_Handle, &msg_print, portMAX_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t *)msg_print, strlen(msg_print), HAL_MAX_DELAY);

		}

}

/************************************************************************************************/
void commandTask_Handler(void *param){
	command_t cmd_t;
	BaseType_t notify_status;

	while(1){
		/* wait until notify from uart Rx isr */
		notify_status = xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

		if(notify_status == pdPASS){
			/* process the command */
			process_command (& cmd_t);

		}
	}
}

/*****************************************************************************************************/
static void process_command(command_t  *pcmd_t){

	int extract_cmd_status;
	extract_cmd_status = extract_command(pcmd_t);

	if(extract_cmd_status){
		switch(current_task_state){
		case sMainMenu:
			xTaskNotify(menu_Handle, (uint32_t)pcmd_t, eSetValueWithOverwrite);
		break;

		case sLedEffect:
			xTaskNotify(led_Handle, (uint32_t)pcmd_t, eSetValueWithOverwrite);
		break;

		case sRtcDateConfig:
		case sRtcMenu:
		case sRtcReport:
		case sRtcTimeConfig:
			xTaskNotify(rtc_Handle, (uint32_t)pcmd_t, eSetValueWithOverwrite);
		break;

		default:

		}
	}

}



/***********************************************************************************************************/
static int extract_command(command_t *pcmd_t){
	uint8_t receive_item, i;
	UBaseType_t  status;

	/* check the command que has data */
	status = uxQueueMessagesWaiting(commandQue_Handle);
	if(! status) return(-1);

	/* get the command from the cmd que */
	do{
		status = xQueueReceive(commandQue_Handle, (void *)&receive_item, 0);
		if(status == pdTRUE) pcmd_t->payload[i++] = receive_item;
	}while(receive_item != '\n');

	/* replace \n with \0 */
	pcmd_t->payload[i-2] = '\0';
	pcmd_t->payload[i-1] = '\0';
	pcmd_t->len = i-2;

	return 1;

}
