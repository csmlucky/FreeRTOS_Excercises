/*
 * led_effect.c
 *
 *  Created on: Sep 26, 2025
 *      Author: csmla
 */

#include "main.h"

#define LED1		(1U << 5)
#define LED2		(1U << 6)
#define LED3		(1U << 7)
#define LED4		(1U << 8)


/*********************************************************************************************************/
void led_effect(uint8_t  n){

	led_effect_stop();
	xTimerStart(ledEffectTimer_Handle[n-1], portMAX_DELAY);


}


/************************************************************************************************************/
void led_effect_stop(void){

	for(int i=0; i<4; i++){
		xTimerStop(ledEffectTimer_Handle[i], portMAX_DELAY);
	}
}

/**************************************************************************************************************/
void turn_off_all_leds(void)
{
	HAL_GPIO_WritePin(GPIOA, LED1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LED2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LED3,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LED4,GPIO_PIN_RESET);
}


/**************************************************************************************************************/
void turn_on_all_leds(void)
{
	HAL_GPIO_WritePin(GPIOA, LED1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED3,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED4,GPIO_PIN_SET);
}

/**************************************************************************************************************/
void turn_on_odd_leds(void)
{
	HAL_GPIO_WritePin(GPIOA, LED1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LED3,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED4,GPIO_PIN_RESET);
}


/**************************************************************************************************************/
void turn_on_even_leds(void)
{
	HAL_GPIO_WritePin(GPIOA, LED1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LED2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED3,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LED4,GPIO_PIN_SET);
}

/**************************************************************************************************************/
void LED_control( int value )
{
  for(int i = 0 ; i < 4 ; i++)
	  HAL_GPIO_WritePin(GPIOA, (1U << (5 + i)), ((value >> i)& 0x1));
}
/*********************************************************************************************************************/
void LED_effect1(void){

	static int flag = 1;
	(flag ^= 1) ? turn_off_all_leds() : turn_on_all_leds();
}

/*******************************************************************************************************************/
void LED_effect2(void){
	static int flag = 1;
	(flag ^= 1) ? turn_on_even_leds() : turn_on_odd_leds();

}

/*******************************************************************************************************************/
void LED_effect3(void){
	static int i = 0;
	LED_control( 0x1 << (i++ % 4) );

}

/*******************************************************************************************************************/
void LED_effect4(void){
	static int i = 0;
	LED_control( 0x8 >> (i++ % 4) );

}








