/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  /*
    Через BLE передается 5 пакетов:

      Текущее параметры системы:
        "0:M:UD:HHmmSS:VVVV\n"
        - режим, delta_on, delta_off(0..9), текущее время, VREF_CAL(mV)
        "1:IIii:HHMM:hhmm\n" 
          - интервал (час., мин.), время начала паузы., время оконч. паузы

      Результаты измерений:
        "2:MM.mm:XX.xx:AA.aa\n" 
          - температуры (.mm, .xx, .aa - десятичная часть, надо умножить на 0.25;)
        "3:VVVV:PPP:L:SS:DDD\n"   
          - VREF_DATA(mV), давление, наличие блокировки, 
            статус(hex), синхронный делитель RTC (399 +-), 
        "4:P:HHMMSS:ME:XE:AE\n"
          - насос вкл./выкл, время последнего старта насоса,
           счетчики ошибок датчиков температуры

    Через BLE может приниматься пакет:
    "CCC:SMOD:[01]:HHmm" - режим 0/1 и интервал в часах и минутах для режима 1
    "CCC:NSET:HHMM:hhmm" - установить начало и конец ночной паузы.
    "CCC:LOCK:[012]" - 1 - остановить насос до night_end, 2 - до команды "CCC:LOCK:0"
    "CCC:DIFF:U:D" - установить разницу температур на вкл/выкл. (0..9).
    "CCC:SCLK:DDD" - установить предделитель RTC (399 +-)
    "CCC:TSET:HHmmSS" - установить время
    "CCC:DSET:YYMMDDW" - установить дату
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "flash.h"
#include "rtc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

//#include "ds18b20.h"
#include "ow_utils.h"
#include "rtc.h"
#include "flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 
JDY-16: CR, LF
JDY-08: no CR no LF
*/
//#define JDY08 1  
#define JDY16   1  

#define NUM_STAT_PACKS  5

//#define DEBUG_SET_SENSORS 1
//#define DEBUG_TEST_OUTPUT 1
//#define DEBUG_SET_BAUD    1

#define PUMP_ON()    LL_GPIO_SetOutputPin(PUMP_GPIO_Port, PUMP_Pin)
#define PUMP_OFF()   LL_GPIO_ResetOutputPin(PUMP_GPIO_Port, PUMP_Pin)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static uint8_t transmit = 0;
static uint8_t rxindex = 0;
static uint8_t flag_cmd = 0;
static uint8_t ble_connected = 0;
static uint8_t adc_scan_complete = 0;
static uint16_t pwm_count = 0;

char info_pack[NUM_STAT_PACKS][PACK_SZ] = { {0}, {0}, {0}, {0}, {0} };
char sndbuf[PACK_SZ];
char rxbuff[PACK_SZ * 2];
char cmdbuf[PACK_SZ];
uint16_t adc_buff[2];

myDevState dev_state = 
  { .celsius_main = 0, .frac_main = 0, .err_curr_main = 0, .err_summ_main = 0,
    .celsius_aux = 0, .frac_aux = 0, .err_curr_aux = 0, .err_summ_aux = 0,
    .celsius_air = 0, .frac_air = 0, .err_curr_air = 0, .err_summ_air = 0,
    .pressure = 0, .vref_data = 0, .state = 0, 
    .hour_last_start = 99, .minute_last_start = 0, .second_last_start = 0, 
    .night = 0, .pump = 0 };

myDevSettings dev_settings = 
  { .signature = CODES_SIGNATURE, .mode = 0, 
    .interval_h = 3, .interval_m = 0,
    .delta_on = DELTA_ON, .delta_off = DELTA_OFF,
    .hour_off_begin = 23, .minute_off_begin = 15,
    .hour_off_end = 7, .minute_off_end = 15,
    .pump_lock = 0, .prescaler = 399, .reserv8_1 = 0x55, .reserv8_2 = 0xaa };

static uint32_t initHours = 12;
static uint32_t initMinutes = 0;
static uint32_t initSeconds = 0;

#define ADC_REPEAT_COUNT 6 // кол-во измерений
static uint16_t a_press[ADC_REPEAT_COUNT];
static uint16_t a_vref[ADC_REPEAT_COUNT];
static SensorData sens;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

void adc_dma_set(void);
void usart_dma_send(char *st);
uint16_t calc_mid(uint16_t * par, uint8_t plen);
uint8_t make_pack(uint8_t pack_num);
uint8_t calc_temp_frac(uint8_t frac);
void led_blink(uint8_t cnt);
void test_output(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Команды BLE JDY-16
#ifdef JDY16
const char ble_baud[] = "AT+BAUD6\r\n"; // 38400
const char ble_name[] = "AT+NAMEPUMP-EAST\r\n";
const char ble_dis[]  = "AT+DISC\r\n";
#else
// JDY08
const char ble_baud[] = "AT+BOUD2"; // 38400
const char ble_name[] = "AT+NAMEPUMP-WEST";
const char ble_dis[]  = "AT+DISC";
#endif

const char cmd_smod[] = "SMOD";
const char cmd_nset[] = "NSET";
const char cmd_lock[] = "LOCK";
const char cmd_diff[] = "DIFF";
const char cmd_tset[] = "TSET";
const char cmd_dset[] = "DSET";
const char cmd_sclk[] = "SCLK";

const char c_delim[]  = ":";
const char c_lf[]  = "\n";
const char c_dot[]  = ".";

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    // !!! trick to enable systick interrupt !!!
  CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk); // HAL_SuspendTick()
  LL_Init1msTick(48000000);
  SET_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);  // HAL_ResumeTick()


  // ?????? RCC_APB2ENR |=RCC_APB2ENR_SYSCFGCOMPEN; // тактирование SYSCFG
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  // MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  uint8_t fl_set = 0; // надо ли сохранять настройки во флеш

  LL_ADC_Enable(ADC1);

  LL_USART_DisableIT_RXNE(USART1);

  for (uint8_t j = 0; j < 5; j++)
  {
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    for (uint16_t i = 0; i < 500; i++)
    {
      delay_us(1000);
    }
    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
    for (uint16_t i = 0; i < 500; i++)
    {
      delay_us(1000);
    }
  }

#ifdef DEBUG_SET_BAUD
  LL_GPIO_ResetOutputPin(PWRC_GPIO_Port, PWRC_Pin); // PWRC=0 - AT mode
  delay(500);
//  strcpy(sndbuf, ble_baud);
  strcpy(sndbuf, ble_name);
  usart_dma_send(sndbuf);
  delay(500);
#endif

  LL_GPIO_SetOutputPin(PWRC_GPIO_Port, PWRC_Pin); // PWRC=1

  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);

  LL_GPIO_SetOutputPin(POWER_GPIO_Port, POWER_Pin); // питание датчиков

#ifdef DEBUG_SET_SENSORS  
  // set tH as sensor number 80,81,...
  if (OWFirst())
  {
    sens.tH = 80;
    sens.tL = 5;
    sens.config = 0x3f;
    do
    {
      //DS1820_write_EEPROM(&sens);
      OWWriteEEPROM(&sens);
      sens.tH++;
    } while (OWNext());
  }
#endif

  // читаем сохраненные параметры из флеш памяти
  uint32_t src_addr = (uint32_t)PARAMETERS_PAGE;
  uint16_t *dst_addr = (void *)&dev_settings;
  uint16_t sign = *(__IO uint16_t *)src_addr;
  if (sign == CODES_SIGNATURE)
  {
    for (uint16_t i = 0; i < (sizeof(myDevSettings) / 2 - 1); i++)
    {
      *dst_addr = *(__IO uint16_t *)src_addr;
      src_addr += 2;
      dst_addr++;
    }    
  }
  else
  {
    fl_set = 1;
  }

  MX_RTC_Init();

  // постоянную блокировку оставляеем, временную сбрасываем
  if (dev_settings.pump_lock == 1)
  {
    dev_settings.pump_lock = 0;
    dev_state.state &= ~STAT_PROGRAN_LOCK;
    fl_set = 1;
  }
  else if (dev_settings.pump_lock == 2)
  {
    dev_state.state |= STAT_PROGRAN_LOCK;
  }

  // текущие параметры системы
  make_pack(0);
  make_pack(1);
  make_pack(2);
  make_pack(3);
  make_pack(4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin); // начало измерений
    
    // Устанавливаем в статус отсутствие отклика от DS18B20
    dev_state.state |= (STAT_SENSOR_MAIN_ERR | STAT_SENSOR_AUX_ERR | STAT_SENSOR_AIR_ERR);
    if (OWFirst())
    {
      do
      {
        if (OWGetParams(MATCH_ROM_CMD, &sens))
        {
#ifdef DEBUG_TEST_OUTPUT
          test_output();
#endif          
          if (sens.crc == sens.crc_calc) // КС совпала
          {
            switch(sens.tH)
            {
              case DS_BATT_MAIN:
                dev_state.celsius_main = sens.celsius;
                dev_state.frac_main = sens.frac;
                // обнуляем текущий счетчик ошибок
                dev_state.err_curr_main = 0;
                // есть отклик - снимаем ошибку в статусе
                dev_state.state &= ~STAT_SENSOR_MAIN_ERR;
                break;
              case DS_BATT_AUX:
                dev_state.celsius_aux = sens.celsius;
                dev_state.frac_aux = sens.frac;            
                dev_state.err_curr_aux = 0;
                dev_state.state &= ~STAT_SENSOR_AUX_ERR;
                break;
              case DS_AIR:
                dev_state.celsius_air = sens.celsius;
                dev_state.frac_air = sens.frac;            
                dev_state.err_curr_air = 0;
                dev_state.state &= ~STAT_SENSOR_AIR_ERR;
                break;
            }
          }
        }
      } while (OWNext());        
    } // сканирование DS18B20 завершено

    // если какой-либо DS18B20 не ответил увеличиваем счетчики ошибок
    if (dev_state.state & STAT_SENSOR_MAIN_ERR)
    {
      dev_state.err_curr_main++;
      if (dev_state.err_summ_main < 99)
      {
        dev_state.err_summ_main++;
      }
    }
    if (dev_state.state & STAT_SENSOR_AUX_ERR)
    {
      dev_state.err_curr_aux++;
      if (dev_state.err_summ_aux < 99)
      {
        dev_state.err_summ_aux++;
      }
    }
    if (dev_state.state & STAT_SENSOR_AIR_ERR)
    {
      dev_state.err_curr_air++;
      if (dev_state.err_summ_air < 99)
      {
        dev_state.err_summ_air++;
      }
    }

    if (dev_state.state & 
      (STAT_SENSOR_MAIN_ERR | STAT_SENSOR_AUX_ERR | STAT_SENSOR_AIR_ERR))
    {
      dev_state.state |= STAT_SENSORS_ERR;
    }
    else
    {
      dev_state.state &= ~STAT_SENSORS_ERR;
    }

    // ADC - опрос датчика давления и VREF
    for (uint8_t i = 0; i < ADC_REPEAT_COUNT; i++)
    {
      adc_dma_set();
      LL_ADC_REG_StartConversion(ADC1);
      while(!adc_scan_complete)
      {
        delay(5);
      }
      a_press[i] = adc_buff[0];
      a_vref[i] = adc_buff[1];
    }
    uint16_t vref_data = calc_mid(a_vref, ADC_REPEAT_COUNT);
    dev_state.vref_data = vref_data;
    uint16_t vref_mv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vref_data, LL_ADC_RESOLUTION_12B);
    uint16_t press_data = calc_mid(a_press, ADC_REPEAT_COUNT);
    dev_state.pressure = 
        __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mv, press_data, LL_ADC_RESOLUTION_12B);

    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin); // конец измерений

    // Обработка езультатов измерений

    // оба датчика батарей выдали результаты?
    if ((dev_state.err_curr_main == 0) && (dev_state.err_curr_main == 0))
    {
      make_pack(2);
    }
    else // датчики батарей не ответили
    {
      if ((dev_state.err_curr_main > 2) || (dev_state.err_curr_aux > 2))
      {
        // датчики батарей не ответили больше 2-х раз подряд
        // отключаем насос
        PUMP_OFF();
        dev_state.pump = 0;
      }
    }

    // Проверяем изменения в системе
    // определяем переключение день/ночь
    uint8_t c_hour = (uint8_t)__LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC));
    uint8_t c_min = (uint8_t)__LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC));
    uint8_t c_sec = (uint8_t)__LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC));

    if (dev_state.night == 0) // сейчас день
    {
      if ((c_hour >= dev_settings.hour_off_begin) && 
          (c_min >= dev_settings.minute_off_begin))
      {
        // перход в ночной режим
        dev_state.night = 1;
        dev_state.state |= STAT_NIGHT;
      }
    }
    else // сейчас ночь
    {
      if ((c_hour == dev_settings.hour_off_end) && 
        (c_min >= dev_settings.minute_off_end))
      {
        // окончание ночного режима по расписанию,
        dev_state.night = 0;
        dev_state.state &= ~STAT_NIGHT;
      }
    }

    // Действия в зависимости от результата (день/ночь)
    if (dev_state.night) // ночь
    {
      if (dev_state.pump)
      {
        PUMP_OFF();
        dev_state.pump = 0;          
      }
      if (dev_settings.pump_lock == 1)
      {
        // при наступлении ночи блокировка типа 1 снимается
        dev_settings.pump_lock = 0;
        dev_state.state &= ~STAT_PROGRAN_LOCK;
        fl_set = 1;
      }
    }
    else // день
    {
      // проверяем условия включения/выключения насоса
      int16_t delta = dev_state.celsius_main - dev_state.celsius_aux;
      int16_t delta_f = dev_state.frac_main - dev_state.frac_aux;
      if (delta_f >= 8)
      {
        delta += 1;
      }
      else if (delta_f <= -8)
      {
        delta -= 1;
      }

      if (dev_state.pump) // насос включен
      {
        if ((dev_settings.pump_lock > 0 ) || (delta <= dev_settings.delta_off))
        {
          // при блокировке или
          // при выравнивании температур батарей - выключаем
          PUMP_OFF();
          dev_state.pump = 0;          
        }
      }
      else // насос не включен
      {
        if (dev_settings.pump_lock == 0) // включаем только при отсутствии блокировки
        {
          if (dev_settings.mode == 0) // - по разности температур
          {
            if (delta >= dev_settings.delta_on)
            {
              dev_state.hour_last_start = c_hour;
              dev_state.minute_last_start = c_min;
              dev_state.second_last_start = c_sec;
              PUMP_ON();
              dev_state.pump = 1;          
            }
          }
          else // mode == 1 - по расписанию
          {
            if (dev_state.hour_last_start == 99)
            {
              // еще не включался, время последнего запуска не установлено
              dev_state.hour_last_start = c_hour;
              dev_state.minute_last_start = c_min;
              dev_state.second_last_start = c_sec;
              PUMP_ON();
              dev_state.pump = 1;          
            }
            else
            {
              // уже был запуск по расписанию
              int16_t h = c_hour - dev_state.hour_last_start;
              if (h < 0)
              {
                h = c_hour + 24 - dev_state.hour_last_start;
              }
              int16_t m = c_min - dev_state.minute_last_start;
              if (m < 0)
              {
                m = c_min + 60 - dev_state.minute_last_start;
              }

              if ((h >= dev_settings.interval_h) && (m >= dev_settings.interval_m))
              {
                // включаем по расписанию
                dev_state.hour_last_start = c_hour;
                dev_state.minute_last_start = c_min;
                dev_state.second_last_start = c_sec;
                PUMP_ON();
                dev_state.pump = 1;          
              }
            } 
          } // по расписанию
        } // dev_settings.pump_lock == 0
      } // не был включен
    } // день/ночь

    // остальные измерения
    make_pack(3);
    // ошибки bs18b20
    make_pack(4);

    delay(200);
    if (dev_settings.pump_lock == 2)
    {
      led_blink(2);
    }
    else if (dev_settings.pump_lock == 1)
    {
      led_blink(1);
    }

    // пауза до 10 сек с проверкой поступления команды и нажатия кнопки
    uint8_t wcnt = 50; // 10 сек
    uint8_t btn_lock = 0;

    if (ble_connected)
    {
      wcnt = 10; // сокращаем паузу до 2 сек
    }
    while (!flag_cmd && !btn_lock && (wcnt-- > 0))
    {
      if (!LL_GPIO_IsInputPinSet(BUTTON_GPIO_Port, BUTTON_Pin))
      {
        btn_lock = 1;
      }
      delay(200);
    }

    if (btn_lock) // было нажатие кнопки?
    {
      // по нажатию кнопки - блокировка/разблокировка насоса
      if (dev_settings.pump_lock == 2)
      {
        dev_settings.pump_lock = 0;
        dev_state.state &= ~STAT_PROGRAN_LOCK;
      }
      else
      {
        dev_settings.pump_lock = 2;
        dev_state.state |= STAT_PROGRAN_LOCK;
      }
      fl_set = 1;
      btn_lock = 0;
      delay(1000);
    }

    if (flag_cmd) // получен пакет из BLE
    {
      char *token, *ptr;
      char strt[24];
      char st1[4];

      uint8_t ii = 0;
      while ((cmdbuf[ii] != '\0') && (cmdbuf[ii] != '\n') 
        && (cmdbuf[ii] != '\r') && (ii < 24) )
      {
        strt[ii] = toupper(cmdbuf[ii]); // копируем с переводом в верхний регистр
        ii++;
      }
      strt[ii] = '\0';

      ptr = strt;
      uint8_t cmd_num = 0;

      token = strsep(&ptr, ":");
      if ( (token != NULL) && (strstr(token, "CC") != NULL))  // есть команда
      {
        token = strsep(&ptr, ":");  // команда
        if (token != NULL)
        {
          if (strcmp(token, cmd_smod) == 0)
          {
            cmd_num = 1;
          }
          else if (strcmp(token, cmd_nset) == 0)
          {
            cmd_num = 2;
          }
          else if (strcmp(token, cmd_lock) == 0)
          {
            cmd_num = 3;
          }
          else if (strcmp(token, cmd_diff) == 0)
          {
            cmd_num = 4;
          }
          else if (strcmp(token, cmd_tset) == 0)
          {
            cmd_num = 5;
          }
          else if (strcmp(token, cmd_dset) == 0)
          {
            cmd_num = 6;
          }
          else if (strcmp(token, cmd_sclk) == 0)
          {
            cmd_num = 7;
          }

          uint8_t par = 0;
          uint16_t par16 = 0;

          if (cmd_num > 0)
          {
            token = strsep(&ptr, ":"); // 1-й параметр
            if (token != NULL)
            {
              switch(cmd_num)
              {
              case 1: // "CCC:SMOD:[01]:HHmm"
                par = atoi(token); // 0, 1
                if (par == 0) // при par == 0 2-й параметр не нужен
                {
                  if (dev_settings.mode > 0)
                  {
                    dev_settings.mode = par;
                    fl_set = 1;
                  }
                }
                else if (par == 1)
                {
                  token = strsep(&ptr, ":"); // 2-й параметр - интервал(HHmm)
                  if ((token != NULL) && (strlen(token) == 4))
                  {
                    strncpy(st1, token, 2);
                    st1[2] = '\0';
                    dev_settings.interval_h = atoi(st1); 
                    strncpy(st1, &token[2], 2);
                    st1[2] = '\0';
                    dev_settings.interval_m = atoi(st1);
                    dev_settings.mode = par;
                    fl_set = 1;
                  }
                }
                break;
              case 2: // "CCC:NSET:HHMM:hhmm"
                if ((strlen(token) == 4))   
                {
                  strncpy(st1, token, 2); // 1-й параметр(HHMM)
                  st1[2] = '\0';
                  dev_settings.hour_off_begin = atoi(st1); 
                  strncpy(st1, &token[2], 2);
                  st1[2] = '\0';
                  dev_settings.minute_off_begin = atoi(st1);                     
                }
                token = strsep(&ptr, ":");
                if ((token != NULL) && (strlen(token) == 4))
                {
                  strncpy(st1, token, 2); // 2-й параметр(hhmm)
                  st1[2] = '\0';
                  dev_settings.hour_off_end = atoi(st1); 
                  strncpy(st1, &token[2], 2);
                  st1[2] = '\0';
                  dev_settings.minute_off_end = atoi(st1);                                         
                }
                fl_set = 1;
                break;
              case 3: // "CCC:LOCK:[012]"
                par = atoi(token);
                if (par < 3)
                {
                  dev_settings.pump_lock = par;
                  dev_state.state &= ~STAT_PROGRAN_LOCK;
                  if (par == 1)
                  {
                    dev_state.state |= STAT_PROGRAN_LOCK;
                  }
                  else if (par == 2)
                  {
                    dev_state.state |= STAT_PROGRAN_LOCK;
                  }
                  fl_set = 1;
                }
                break;
              case 4: // "CCC:DIFF:U:D"
                par = atoi(token);
                if ((par > 0) && (par <= 9))  // 1..9 градусов
                {
                  dev_settings.delta_on = par;
                }
                token = strsep(&ptr, ":");
                if (token != NULL)
                {
                  par = atoi(token);
                  if ((par > 0) && (par <= 9))  // 1..9 градусов
                  {
                    dev_settings.delta_off = par;
                  }
                }
                fl_set = 1;
                break;
              case 5: // "CCC:TSET:HHmmSS"
                if (strlen(token) == 6)
                {
                  RTC_set_time_from_st(token);
                }
                // заодно обнуляем счетчики ошибок
                dev_state.err_summ_main = 0;
                dev_state.err_summ_aux = 0;
                dev_state.err_summ_air = 0;
                break;
              case 6: // "CCC:DSET:YYMMDDW" - дата пока не нужна
                if (strlen(token) == 7)
                {
                  RTS_set_date_from_st(token);
                }
                break;
              case 7: // "CCC:SCLK:DDD"
                par16 = atoi(token);
                if ((par16 >= 360) && (par16 <= 440))
                {
                  initHours = LL_RTC_TIME_GetHour(RTC);
                  initMinutes = LL_RTC_TIME_GetMinute(RTC);
                  initSeconds = LL_RTC_TIME_GetSecond(RTC);
                  dev_settings.prescaler = (uint32_t)par16;
                  MX_RTC_Init();
                  fl_set = 1;
                }
                break;
              }
            }
          }
        }
      }
      flag_cmd = 0;
    } // конец обработки команды
            
    if (fl_set) // сохранять параметры
    {
      save_parameters_to_flash((uint16_t *)&dev_settings, 
        sizeof(myDevSettings) / 2 - 1);
      make_pack(1);
      fl_set = 0;
    }

    make_pack(0);

    while(transmit) {};
    uint8_t np = 0;
    while (ble_connected && (np < NUM_STAT_PACKS))
    {
      strcpy(sndbuf, info_pack[np++]);
      usart_dma_send(sndbuf);
      delay(200);
      while(transmit) {};
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {

  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_PWR_EnableBkUpAccess();
  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI)
  {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  }
  LL_RCC_EnableRTC();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(48000000);
  LL_SetSystemCoreClock(48000000);
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**ADC GPIO Configuration
  PA7   ------> ADC_IN7
  */
  GPIO_InitStruct.Pin = PRESSURE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(PRESSURE_GPIO_Port, &GPIO_InitStruct);

  /* ADC DMA Init */

  /* ADC Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_7);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_VREFINT);
  /** Configure Internal Channel
  */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  ADC_InitStruct.Clock = LL_ADC_CLOCK_ASYNC;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_71CYCLES_5);
  /* USER CODE BEGIN ADC_Init 2 */

  uint32_t wait_loop_index = ((LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32) >> 1);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  LL_ADC_StartCalibration(ADC1);
  while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0) {}

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  LL_RTC_InitTypeDef RTC_InitStruct = {0};
  LL_RTC_TimeTypeDef RTC_TimeStruct = {0};
  LL_RTC_DateTypeDef RTC_DateStruct = {0};

  /* Peripheral clock enable */
  LL_RCC_EnableRTC();

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC and set the Time and Date
  */
  RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct.AsynchPrescaler = 99;
  RTC_InitStruct.SynchPrescaler = dev_settings.prescaler; // 399 - default
  LL_RTC_Init(RTC, &RTC_InitStruct);
  LL_RTC_SetAsynchPrescaler(RTC, 99);
  LL_RTC_SetSynchPrescaler(RTC, dev_settings.prescaler); // 399 - default
  /** Initialize RTC and set the Time and Date
  */
  RTC_TimeStruct.Hours = initHours;
  RTC_TimeStruct.Minutes = initMinutes;
  RTC_TimeStruct.Seconds = initSeconds;
  LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_TimeStruct);
  RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_MONDAY;
  RTC_DateStruct.Month = LL_RTC_MONTH_DECEMBER;
  RTC_DateStruct.Year = 20;
  LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_DateStruct);
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16);

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  TIM_InitStruct.Prescaler = 47;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM16, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM16);
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 DMA Init */

  /* USART1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 38400;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_DisableIT_TXE(USART1);
  LL_USART_DisableIT_TC(USART1);
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(PWRC_GPIO_Port, PWRC_Pin);

  /**/
  LL_GPIO_ResetOutputPin(POWER_GPIO_Port, POWER_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(PUMP_GPIO_Port, PUMP_Pin);

  /**/
  LL_GPIO_SetOutputPin(DS18C20_GPIO_Port, DS18C20_Pin);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE1);

  /**/
  LL_GPIO_SetPinPull(PWM_IN_GPIO_Port, PWM_IN_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(STAT_GPIO_Port, STAT_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(PWM_IN_GPIO_Port, PWM_IN_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(STAT_GPIO_Port, STAT_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PWRC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(PWRC_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = POWER_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(POWER_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PUMP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(PUMP_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DS18C20_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DS18C20_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI0_1_IRQn, 0);
  NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
void usart_dma_send(char * buf) //(uint32_t)buf strlen(buf)
{
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)buf);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&USART1->TDR);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, (uint32_t)strlen(buf));

  //  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
  //  dma_enable_channel(DMA1, DMA_CHANNEL2);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  //  usart_enable_tx_dma(USART1);
  LL_USART_EnableDMAReq_TX(USART1);

  transmit = 1;
}

// usart1 dma interrupt
void dma1_channel2_callback(void)
{
    if (LL_DMA_IsActiveFlag_TC2(DMA1))
    {
        LL_DMA_ClearFlag_TC2(DMA1);
    }
    //dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    //usart_disable_tx_dma(USART1);
    LL_USART_DisableDMAReq_TX(USART1);
    //dma_disable_channel(DMA1, DMA_CHANNEL2);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

    transmit = 0;

}


void usart1_isr_callback(void)
{
  char data;
  // если не сбросиить ошибку переполнения - задолбает прерываниями!
  //tmp = USART1->CR1;
  if (LL_USART_IsActiveFlag_ORE(USART1))
  {
    data = LL_USART_ReceiveData8(USART1);
    LL_USART_ClearFlag_ORE(USART1);
  }

  if (LL_USART_IsEnabledIT_RXNE(USART1) &&
            LL_USART_IsActiveFlag_RXNE(USART1))
  {
    data = LL_USART_ReceiveData8(USART1);
    if (rxindex < PACK_SZ)
    {
      rxbuff[rxindex++] = data;
      if ((data == '\n') || (rxindex == PACK_SZ))
      {
        rxbuff[rxindex] = '\0';
        strcpy(cmdbuf, rxbuff);
        flag_cmd = 1;
        rxindex = 0;
        rxbuff[0] = '\0';
      }
    }
  }
        
  if (LL_USART_IsEnabledIT_TXE(USART1) &&
        LL_USART_IsActiveFlag_TXE(USART1))
  {
    LL_USART_DisableIT_TXE(USART1);
  }
}

// ADC
void adc_dma_set(void)
{
  adc_scan_complete = 0;
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_7);
  //LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_TEMPSENSOR);
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);
  //LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), 
  //          LL_ADC_PATH_INTERNAL_VREFINT|LL_ADC_PATH_INTERNAL_TEMPSENSOR);

    //dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC1_DR);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);
    //dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_buff);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)adc_buff);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);

  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_71CYCLES_5);

  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    //dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    //dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL1);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void dma1_channel1_callback(void)
{
  if (LL_DMA_IsActiveFlag_TC1(DMA1)) // Tranfer complete flag
  {
    LL_DMA_ClearFlag_TC1(DMA1);
    // сканирование каналов закончено
  }
    
  if (LL_DMA_IsActiveFlag_TE1(DMA1)) // Tranfer error flag
  {
    LL_DMA_ClearFlag_TE1(DMA1);
  }

  adc_scan_complete = 1;
}

void exti1_isr_callback(void)
{

  if (LL_GPIO_IsInputPinSet(STAT_GPIO_Port, STAT_Pin)) // __|
  {
    ble_connected = 1;
    NVIC_EnableIRQ(USART1_IRQn);
    LL_USART_EnableIT_RXNE(USART1);
  }
  else                                // |__
  {
    ble_connected = 0;
    LL_USART_DisableIT_RXNE(USART1);
    NVIC_DisableIRQ(USART1_IRQn);
  }
}

void exti0_isr_callback(void)
{
  if (LL_GPIO_IsInputPinSet(PWM_IN_GPIO_Port, PWM_IN_Pin))
  {
    pwm_count++;
  }
}

// ============================================================
uint8_t make_pack(uint8_t pack_num)
{
  if (pack_num >= NUM_STAT_PACKS)
  {
    return 1;
  }

  strcpy(info_pack[pack_num], itoa_m0(pack_num, 10, 1));
  strcat(info_pack[pack_num], c_delim);

  switch (pack_num)
  {
    case 0: // "0:M:UD:HHmmSS:VVVV\n"
      // режим, delta_on, delta_off, тек. время, VREFINT_CAL
      strcat(info_pack[pack_num], itoa_m0(dev_settings.mode, 10, 1));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_settings.delta_on, 10, 1));
      strcat(info_pack[pack_num], itoa_m0(dev_settings.delta_off, 10, 1));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], RTC_get_time_to_str());
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0((*VREFINT_CAL_ADDR), 10, 4));
      break;
    case 1: // "1:IIii:HHMM:hhmm\n"
      strcat(info_pack[pack_num], itoa_m0(dev_settings.interval_h, 10, 2));
      strcat(info_pack[pack_num], itoa_m0(dev_settings.interval_m, 10, 2));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_settings.hour_off_begin, 10, 2));
      strcat(info_pack[pack_num], itoa_m0(dev_settings.minute_off_begin, 10, 2));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_settings.hour_off_end, 10, 2));
      strcat(info_pack[pack_num], itoa_m0(dev_settings.minute_off_end, 10, 2));
      break;
    case 2: // "2:MM.mm:XX.xx:AA.aa\n"
      //    - температуры (.mm, .xx, .aa - десятичная часть, надо умножить на 0.25;)
      strcat(info_pack[pack_num], itoa_m0(dev_state.celsius_main, 10, 2));
      strcat(info_pack[pack_num], c_dot);
      strcat(info_pack[pack_num], itoa_m0(calc_temp_frac(dev_state.frac_main), 10, 2));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_state.celsius_aux, 10, 2));
      strcat(info_pack[pack_num], c_dot);
      strcat(info_pack[pack_num], itoa_m0(calc_temp_frac(dev_state.frac_aux), 10, 2));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_state.celsius_air, 10, 2));
      strcat(info_pack[pack_num], c_dot);
      strcat(info_pack[pack_num], itoa_m0(calc_temp_frac(dev_state.frac_air), 10, 2));
      break;
    case 3: // "3:VVVV:PPP:L:SS:DDD\n"   
      strcat(info_pack[pack_num], itoa_m0(dev_state.vref_data, 10, 4));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_state.pressure, 10, 3));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_settings.pump_lock, 10, 1));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_state.state, 16, 2));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_settings.prescaler, 10, 3));
      break;
    case 4: // "4:P:HHMMSS:ME:XE:AE\n"
      strcat(info_pack[pack_num], itoa_m0(dev_state.pump, 10, 1));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_state.hour_last_start, 10, 2));
      strcat(info_pack[pack_num], itoa_m0(dev_state.minute_last_start, 10, 2));
      strcat(info_pack[pack_num], itoa_m0(dev_state.second_last_start, 10, 2));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_state.err_summ_main, 10, 2));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_state.err_summ_aux, 10, 2));
      strcat(info_pack[pack_num], c_delim);
      strcat(info_pack[pack_num], itoa_m0(dev_state.err_summ_air, 10, 2));
      break;
  }

  strcat(info_pack[pack_num], c_lf);
  return 0;
}


void delay_us(uint16_t duration)
{
    LL_TIM_SetCounter(TIM16, 0);
    //timer_set_period(TIM16, 0xffff);  // ARR
    LL_TIM_EnableCounter(TIM16);
    uint16_t t = 0;
    while (t < duration)
    {
        t = (uint16_t)LL_TIM_GetCounter(TIM16);
    }
    LL_TIM_DisableCounter(TIM16);
}

void delay(uint64_t duration) {
    const uint64_t until = millis() + duration;
    while (millis() < until) { }
}


// itoa_m с добавлением ведущих нулей до len
char * itoa_m0(int val, int base, int len)
{
  static char buf[32] = {0};

  int i = 30;
  unsigned int uval = abs(val);

  for(; uval && i ; --i, uval /= base)
  {
    buf[i] = "0123456789abcdef"[uval % base];
  }
  int ll = 30 - i;
  if (ll == 0)
  {
    buf[i--] = '0';
    ll = 1;
  }
  for (int n = 0; n < (len - ll); n++)
  {
    buf[i--] = '0';
  }
  if (val < 0)
  {
    if (buf[i + 1] == '0')
    {
      i++;
    }
    buf[i] = '-';
    i--;
  }
  return &buf[i+1];
}

// вычисляем среднее значение измеѸений с отбрасыванием min и max
// количество измерений plen не должно быть меньше 4
uint16_t calc_mid(uint16_t * par, uint8_t plen)
{
    uint8_t imin = 0;
    uint8_t imax = 0;
    uint16_t resmid = 0;
    
    for (uint8_t i = 1; i < plen; i++)
    {
        if (par[i] >= par[imax])
        {
            imax = i;
        }

        if (par[i] < par[imin])
        {
            imin = i;
        }
    }
    for (uint8_t i = 0; i < plen; i++)
    {
        if ((i != imin) && (i != imax))
        {
            resmid += par[i];
        }
    }
        
    return (resmid / (plen - 2));
}       

uint8_t calc_temp_frac(uint8_t frac)
{
  uint16_t tmp = (frac >> 2); // 0, 4, 8, 12 > 0, 1, 2, 3
  tmp = tmp * 100 / 4;
  return (uint8_t)tmp;
}

void led_blink(uint8_t cnt)
{
  for (uint8_t i = 0; i < cnt; i++)
  {
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    delay(200);
    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
    delay(200);
  }
}

void test_output(void)
{
  strcpy(cmdbuf, itoa_m0(sens.celsius, 10, 1));
  strcat(cmdbuf, ", ");
  strcat(cmdbuf, itoa_m0(sens.frac, 10, 1));
  strcat(cmdbuf, ", ");
  strcat(cmdbuf, itoa_m0(sens.tH, 10, 1));
  strcat(cmdbuf, ", ");
  strcat(cmdbuf, itoa_m0(sens.tL, 10, 1));
  strcat(cmdbuf, ", ");
  strcat(cmdbuf, itoa_m0(sens.config, 16, 1));
  strcat(cmdbuf, ", ");
  strcat(cmdbuf, itoa_m0(sens.res_0xff, 16, 1));
  strcat(cmdbuf, ", ");
  strcat(cmdbuf, itoa_m0(sens.res_none, 16, 1));
  strcat(cmdbuf, ", ");
  strcat(cmdbuf, itoa_m0(sens.res_0x10, 16, 1));
  strcat(cmdbuf, ", ");
  strcat(cmdbuf, itoa_m0(sens.crc, 16, 1));
  strcat(cmdbuf, ", ");
  strcat(cmdbuf, itoa_m0(sens.crc_calc, 16, 1));
  strcat(cmdbuf, "\n");
  while(transmit) {};
  strcpy(sndbuf, cmdbuf);
  usart_dma_send(sndbuf);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
