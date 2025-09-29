/*
 * task_handlers.c
 *
 *  Created on: Sep 26, 2025
 *      Author: csmla
 */

#include "main.h"

void process_command (command_t *pcmd_t);
int extract_command(command_t *pcmd_t);

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
				xTaskNotify(rtc_Handle, 0, eNoAction);
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
			continue;
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
				continue;
			}

		}else{
			/* print option is invalid */
			xQueueSend(printQue_Handle, &msg_invalid, 0);

		}
		/*update state variable */
		current_task_state = sMainMenu;

		/* Notify menu task */
		xTaskNotify(menu_Handle,0,eNoAction);

		}

}


/********************************************************************************************/
uint8_t getnumber(uint8_t *p , int len)
{

	int value ;

	if(len > 1)
	   value =  ( ((p[0]-48) * 10) + (p[1] - 48) );
	else
		value = p[0] - 48;

	return value;

}



/*****************************************************************************************************/
void rtcTask_Handler(void *param){

	const char *msg_rtc_prompts[] = {
	    "========================\n"
	    "|         RTC          |\n"
	    "========================\n",

	    "Configure Time            ----> 0\n"
	    "Configure Date            ----> 1\n"
	    "Enable reporting          ----> 2\n"
	    "Exit                      ----> 4\n"
	    "Enter your choice here : ",

	    "Enter hour(1-12):",
	    "Enter minutes(0-59):",
	    "Enter seconds(0-59):",

	    "Enter date(1-31):",
	    "Enter month(1-12):",
	    "Enter day(1-7 sun:1):",
	    "Enter year(0-99):",

	    "Configuration successful\n",
	    "Enable time&date reporting(y/n)?: "
	};

	uint32_t cmd_addr;
	command_t *pcmd_t;
	uint8_t menu_code;

	static uint8_t rtc_state = 0;
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;

	while(1){

		/*Wait for notification (Notify wait) */
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

		/* Print the menu and show current date and time information */
		xQueueSend(printQue_Handle, &msg_rtc_prompts[RTC_HEADER], 0);
		show_time_date();
		xQueueSend(printQue_Handle, &msg_rtc_prompts[RTC_MENU], 0);


		while(current_task_state != sMainMenu){

			/*Wait for command notification (Notify wait) */
			xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);
			pcmd_t = (command_t *) cmd_addr;

			switch(current_task_state){
				case sRtcMenu:
				/*process RTC menu commands */
				if(pcmd_t->len == 1){

					menu_code = pcmd_t->payload[0] - 48;
					switch(menu_code)
					{
					case 0:
						current_task_state = sRtcTimeConfig;
						xQueueSend(printQue_Handle,&msg_rtc_prompts[RTC_HOUR],portMAX_DELAY);
					break;

					case 1:
						current_task_state = sRtcDateConfig;
						xQueueSend(printQue_Handle,&msg_rtc_prompts[RTC_DATE],portMAX_DELAY);
						break;
					case 2 :
						current_task_state = sRtcReport;
						xQueueSend(printQue_Handle,&msg_rtc_prompts[RTC_REPORT_PROMPT],portMAX_DELAY);
						break;
					case 3 :
						current_task_state = sMainMenu;
						break;
					default:
						current_task_state = sMainMenu;
						xQueueSend(printQue_Handle,&msg_invalid,portMAX_DELAY);
					}
				}else{
					current_task_state = sMainMenu;
					xQueueSend(printQue_Handle,&msg_invalid,portMAX_DELAY);
				}
			break;

			case sRtcTimeConfig:
				/*get hh, mm, ss infor and configure RTC */
				/*take care of invalid entries */
				switch(rtc_state){

					case HH_CONFIG:
						uint8_t hour = getnumber(pcmd_t->payload , pcmd_t->len);
						time.Hours = hour;
						rtc_state = MM_CONFIG;
						xQueueSend(printQue_Handle,&msg_rtc_prompts[RTC_MIN],portMAX_DELAY);
						break;
					case MM_CONFIG:
						uint8_t min = getnumber(pcmd_t->payload , pcmd_t->len);
						time.Minutes = min;
						rtc_state = SS_CONFIG;
						xQueueSend(printQue_Handle,&msg_rtc_prompts[RTC_SEC],portMAX_DELAY);
						break;
					case SS_CONFIG:
						uint8_t sec = getnumber(pcmd_t->payload , pcmd_t->len);
						time.Seconds = sec;
						if(!validate_rtc_information(&time,NULL))
						{
							rtc_configure_time(&time);
							xQueueSend(printQue_Handle,&msg_rtc_prompts[RTC_CONF_OK],portMAX_DELAY);
							show_time_date();
						}else
							xQueueSend(printQue_Handle,&msg_invalid,portMAX_DELAY);

						current_task_state = sMainMenu;
						rtc_state = 0;
						break;
					}

			break;

			case sRtcDateConfig:

			/*get date, month, day , year info and configure RTC */

			/*take care of invalid entries */
				switch(rtc_state){


					case DATE_CONFIG:
						uint8_t d = getnumber(pcmd_t->payload , pcmd_t->len);
						date.Date = d;
						rtc_state = MONTH_CONFIG;
						xQueueSend(printQue_Handle,&msg_rtc_prompts[RTC_MONTH],portMAX_DELAY);
						break;
					case MONTH_CONFIG:
						uint8_t month = getnumber(pcmd_t->payload , pcmd_t->len);
						date.Month = month;
						rtc_state = DAY_CONFIG;
						xQueueSend(printQue_Handle,&msg_rtc_prompts[RTC_DOW],portMAX_DELAY);
						break;
					case DAY_CONFIG:
						uint8_t day = getnumber(pcmd_t->payload , pcmd_t->len);
						date.WeekDay = day;
						rtc_state = YEAR_CONFIG;
						xQueueSend(printQue_Handle,&msg_rtc_prompts[RTC_YEAR],portMAX_DELAY);
						break;
					case YEAR_CONFIG:
						uint8_t year = getnumber(pcmd_t->payload , pcmd_t->len);
						date.Year = year;

						if(!validate_rtc_information(NULL,&date))
						{
							rtc_configure_date(&date);
							xQueueSend(printQue_Handle,&msg_rtc_prompts[RTC_CONF_OK],portMAX_DELAY);
							show_time_date();
						}else
							xQueueSend(printQue_Handle,&msg_invalid,portMAX_DELAY);

						current_task_state = sMainMenu;
						rtc_state = 0;
						break;
					}


			break;

			case sRtcReport:
				/*enable or disable RTC current time reporting over ITM printf */
				if(pcmd_t->len == 1)
				{
					if(pcmd_t->payload[0] == 'y'){
						if(xTimerIsTimerActive(rtc_timer) == pdFALSE)
							xTimerStart(rtc_timer,portMAX_DELAY);
					}else if (pcmd_t->payload[0] == 'n'){
						xTimerStop(rtc_timer,portMAX_DELAY);
					}else{
						xQueueSend(printQue_Handle,&msg_invalid,portMAX_DELAY);
					}

				}else
					xQueueSend(printQue_Handle,&msg_invalid,portMAX_DELAY);

				current_task_state = sMainMenu;
			break;

			default:

			}// switch end

		} //while end


		/*Notify menu task */
		xTaskNotify(menu_Handle,0,eNoAction);

		/*update state variable */
		current_task_state = sMainMenu;

		/* Notify menu task */
		xTaskNotify(menu_Handle,0,eNoAction);

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
void process_command(command_t  *pcmd_t){

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
int extract_command(command_t *pcmd_t){
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
