/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void RunStop_Pump(int action);
uint16_t I2CGetData(uint16_t ADDRESS);
uint16_t CurrentPower5A(void);
uint8_t WaterLevel(const uint16_t raw1, const uint16_t raw2);
uint16_t BatteriCalculation(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Current_ADC_Pin GPIO_PIN_0
#define Current_ADC_GPIO_Port GPIOC
#define Battery_Pin GPIO_PIN_3
#define Battery_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define Detect2_Pin GPIO_PIN_14
#define Detect2_GPIO_Port GPIOD
#define Detect1_Pin GPIO_PIN_15
#define Detect1_GPIO_Port GPIOD
#define bilge_pump_Pin GPIO_PIN_4
#define bilge_pump_GPIO_Port GPIOD
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define NoWater 1450
#define NoWater2 1200

#define ADC_AVG_SIZE 10
#define SCALING_FACTOR 190.0

/* Error messages */
#define ErrorADC1 490
#define ErrorWaterDetect1 491
#define ErrorWaterDetect2 492
#define ErrorWaterLevel1 493
#define ErrorWaterLevel2 494
#define ErrorADCCurrentPollConversion 5400
#define ErrorI2CTransmit 5100
#define ErrorI2CReceive 5110
#define ErrorBattery 5210
#define ErrorADC2 520
#define ErrorPumpMsg 530

#define SHUNT_VOLTAGE_REGISTER 0x01 		// make it listen to shunt voltage
#define SENSOR_ADDRESS1 0x40<<1	// Current sensor address unbridged
#define SENSOR_ADDRESS2 0x41<<1 // Current sensor address A0 bridged
#define SENSOR_ADDRESS3 0x44<<1	// Current sensor address A1 bridged
#define SENSOR_ADDRESS4 0x45<<1 // Current sensor address A0 bridged A1 bridged
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
