/*
 * sensor.c
 *
 *  Created on: 9 Nov 2017
 *      Author: tinova
 */

#include "sensor.h"

uint8_t SSBtn(void)
{
	uint8_t FlagDetect=0;
	if(HAL_GPIO_ReadPin(SSBtn_GPIO_Port,SSBtn_Pin)== GPIO_PIN_RESET) {
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(SSBtn_GPIO_Port,SSBtn_Pin)== GPIO_PIN_RESET) {
			while(HAL_GPIO_ReadPin(SSBtn_GPIO_Port,SSBtn_Pin)== GPIO_PIN_RESET);
			FlagDetect=1;
		}
	}
	return FlagDetect;
}
uint8_t RstBtn(void)
{
	uint8_t FlagDetect=0;
	if(HAL_GPIO_ReadPin(RstBtn_GPIO_Port,RstBtn_Pin)== GPIO_PIN_RESET) {
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(RstBtn_GPIO_Port,RstBtn_Pin)== GPIO_PIN_RESET) {
			while(HAL_GPIO_ReadPin(RstBtn_GPIO_Port,RstBtn_Pin)== GPIO_PIN_RESET);
			FlagDetect=1;
		}
	}
	return FlagDetect;
}
uint8_t Ssr1(void)
{
	uint8_t FlagDetect=0;
	if(HAL_GPIO_ReadPin(Ssr1_GPIO_Port,Ssr1_Pin)== GPIO_PIN_RESET) {
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(Ssr1_GPIO_Port,Ssr1_Pin)== GPIO_PIN_RESET) {
			while(HAL_GPIO_ReadPin(Ssr1_GPIO_Port,Ssr1_Pin)== GPIO_PIN_RESET);
			FlagDetect=1;
		}
	}
	return FlagDetect;
}
uint8_t Ssr2(void)
{
	uint8_t FlagDetect=0;
	if(HAL_GPIO_ReadPin(Ssr2_GPIO_Port,Ssr2_Pin)== GPIO_PIN_RESET) {
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(Ssr2_GPIO_Port,Ssr2_Pin)== GPIO_PIN_RESET) {
			while(HAL_GPIO_ReadPin(Ssr2_GPIO_Port,Ssr2_Pin)== GPIO_PIN_RESET);
			FlagDetect=1;
		}
	}
	return FlagDetect;
}
uint8_t Ssr3(void)
{
	uint8_t FlagDetect=0;
	if(HAL_GPIO_ReadPin(Ssr3_GPIO_Port,Ssr3_Pin)== GPIO_PIN_RESET) {
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(Ssr3_GPIO_Port,Ssr3_Pin)== GPIO_PIN_RESET) {
			while(HAL_GPIO_ReadPin(Ssr3_GPIO_Port,Ssr3_Pin)== GPIO_PIN_RESET);
			FlagDetect=1;
		}
	}
	return FlagDetect;
}
