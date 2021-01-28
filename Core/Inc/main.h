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
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_rtc.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    int8_t celsius_main;  // основная батарея
    uint8_t frac_main;
    uint8_t err_curr_main;
    uint16_t err_summ_main;
    int8_t celsius_aux;  // дополнительная батарея
    uint8_t frac_aux;
    uint8_t err_curr_aux;
    uint16_t err_summ_aux;
    int8_t celsius_air;  // температура воздуха
    uint8_t frac_air;
    uint8_t err_curr_air;
    uint16_t err_summ_air;
    uint16_t pressure;
    uint16_t vref_data;
    uint8_t state;
    uint8_t hour_last_start;
    uint8_t minute_last_start;
    uint8_t second_last_start;
    uint8_t night;
    uint8_t pump;
} myDevState;

typedef struct {
    uint16_t signature;
    uint8_t mode;             // режим работы: 0 - по отклонению, 1 - по часам
    uint8_t interval_h;       // интервал включения при режиме 1, часы
    uint8_t interval_m;       // интервал включения при режиме 1, минуты
    uint8_t delta_on;         // разница температур включения насоса в режиме 0
    uint8_t delta_off;        // разница температур выключения в обоих режимах
    uint8_t hour_off_begin;   // время начала ночной паузы
    uint8_t minute_off_begin;
    uint8_t hour_off_end;     // время окончания ночной паузы
    uint8_t minute_off_end;
    uint8_t pump_lock;        // 0 - нет блокировки, 1 - до окончания паузы, 2 - до отмены
    uint32_t prescaler;
    uint8_t reserv8_1;
    uint8_t reserv8_2;
} myDevSettings;

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
uint64_t millis(void);
void delay_us(uint16_t duration);
void delay(uint64_t duration);
char * itoa_m0(int val, int base, int len);

void dma1_channel1_callback(void);
void dma1_channel2_callback(void);
void usart1_isr_callback(void);
void exti0_isr_callback(void);
void exti1_isr_callback(void);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_IN_Pin LL_GPIO_PIN_0
#define PWM_IN_GPIO_Port GPIOA
#define PWM_IN_EXTI_IRQn EXTI0_1_IRQn
#define PWRC_Pin LL_GPIO_PIN_1
#define PWRC_GPIO_Port GPIOA
#define POWER_Pin LL_GPIO_PIN_2
#define POWER_GPIO_Port GPIOA
#define LED_Pin LL_GPIO_PIN_3
#define LED_GPIO_Port GPIOA
#define PUMP_Pin LL_GPIO_PIN_4
#define PUMP_GPIO_Port GPIOA
#define DS18C20_Pin LL_GPIO_PIN_5
#define DS18C20_GPIO_Port GPIOA
#define BUTTON_Pin LL_GPIO_PIN_6
#define BUTTON_GPIO_Port GPIOA
#define PRESSURE_Pin LL_GPIO_PIN_7
#define PRESSURE_GPIO_Port GPIOA
#define STAT_Pin LL_GPIO_PIN_1
#define STAT_GPIO_Port GPIOB
#define STAT_EXTI_IRQn EXTI0_1_IRQn
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */
#define DS_BATT_AUX   80
#define DS_BATT_MAIN  81
#define DS_AIR        82

#define PACK_SZ     24
#define DELTA_ON    6
#define DELTA_OFF   1

#define STAT_NIGHT            0x01
#define STAT_SENSORS_ERR      0x02
#define STAT_PROGRAN_LOCK     0x04
#define STAT_RESERV           0x08
#define STAT_SENSOR_AIR_ERR   0x20
#define STAT_SENSOR_AUX_ERR   0x40
#define STAT_SENSOR_MAIN_ERR  0x80

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
