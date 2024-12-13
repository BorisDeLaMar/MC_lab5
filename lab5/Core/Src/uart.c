#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"

#define MAX_U 4095
#define MAX_GRAD 270
#define NUM_OF_DMA_ELEMENTS 2
#define PRESICION 10

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;

uint16_t value[NUM_OF_DMA_ELEMENTS] = {0, 0};
uint32_t servo_pos = 0;
uint32_t external_servo_pos = 0;
uint32_t external_servo_pos_prev = 0;
uint32_t u = 0;
float e = 0;
float e_prev = 0;
float e_sum = 0;

uint32_t pid(float k1, float k2, float k3, float e, float e_prev) {
	return k1*e + k2*(e_sum*0.0001) + k3*(abs(e-e_prev))*1000.0f; //freq = 1000
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM4) {
		HAL_ADC_Start_DMA(&hadc1, value, NUM_OF_DMA_ELEMENTS);
		//HAL_UART_Transmit(&huart2, value, NUM_OF_DMA_ELEMENTS*4, 20);
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2) {
		int err = (int)external_servo_pos - (int)servo_pos;
		if(err <= 0)
			HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET);

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, u);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	if(hadc1->Instance == ADC1)
	{
		int32_t flag = value;
		servo_pos = (float)abs(MAX_U - value[1])/(float)MAX_U*MAX_GRAD;
		external_servo_pos = (float)abs(MAX_U - value[0])/(float)MAX_U*MAX_GRAD;

		if(external_servo_pos > MAX_GRAD-6)
			external_servo_pos = MAX_GRAD-6;

		e = abs(external_servo_pos - servo_pos);
		u = pid(2, 30, 0.8, e, e_prev);

		if((e < PRESICION) && (external_servo_pos_prev - external_servo_pos) == 0) {
			u = 0;
			e = 0;
			e_sum = 0;
		}

		e_prev = e;
		external_servo_pos_prev = external_servo_pos;
		e_sum += e;

		uint32_t answer[5] = {0xAAAAAAAA, servo_pos, external_servo_pos, u, e};
		HAL_UART_Transmit(&huart2, answer, 20, 20);
	}
}
