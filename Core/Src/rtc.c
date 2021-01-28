#include "stm32f0xx_ll_rtc.h"

#include <string.h>
#include <stdlib.h>

#include "rtc.h"
#include "main.h"

void RTC_change_time(uint8_t new_hh, uint8_t new_mm, uint8_t new_ss)
{
  	LL_RTC_TimeTypeDef RTC_TimeStruct = {0};

  	RTC_TimeStruct.TimeFormat = LL_RTC_TIME_FORMAT_AM_OR_24;
  	RTC_TimeStruct.Hours = new_hh;
  	RTC_TimeStruct.Minutes = new_mm;
  	RTC_TimeStruct.Seconds = new_ss;
  	LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_TimeStruct);
}

void RTC_change_date(uint8_t new_yy, uint8_t new_mm, uint8_t new_dd, uint8_t new_dow)
{
	LL_RTC_DateTypeDef RTC_DateStruct = {0};

  	RTC_DateStruct.WeekDay = new_dow;
  	RTC_DateStruct.Month = new_mm;
  	RTC_DateStruct.Year = new_yy;
  	LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_DateStruct);
}

uint8_t RTC_set_time_from_st(char *st_time)
{
	if (strlen(st_time) != 6)	// HHmmSS
	{
		return 0;
	}

	char stmp[4];

	strncpy(stmp, st_time, 2);
	stmp[2] = '\0';
	uint8_t hh = atoi(stmp);
	strncpy(stmp, (st_time + 2), 2);
	stmp[2] = '\0';
	uint8_t mm = atoi(stmp);
	strncpy(stmp, (st_time + 4), 2);
	stmp[2] = '\0';
	uint8_t ss = atoi(stmp);

	RTC_change_time(hh, mm, ss);

	return 1;
}

uint8_t RTS_set_date_from_st(char *st_date)
{
	if (strlen(st_date) != 7)	// YYMMDDW
	{
		return 0;
	}

	char stmp[4];

	strncpy(stmp, st_date, 2);
	stmp[2] = '\0';
	uint8_t yy = atoi(stmp);
	strncpy(stmp, (st_date + 2), 2);
	stmp[2] = '\0';
	uint8_t mm = atoi(stmp);
	strncpy(stmp, (st_date + 4), 2);
	stmp[2] = '\0';
	uint8_t dd = atoi(stmp);
	stmp[0] = st_date[6];
	stmp[1] = '\0';
	uint8_t dow = atoi(stmp);

	RTC_change_date(yy, mm, dd, dow);

	return 1;
}


char * RTC_get_time_to_str(void)
{
	static char buf[10] = {0};

	uint32_t tmp;

	tmp = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC));
	strcpy(buf, itoa_m0(tmp, 10, 2));
	tmp = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC));
	strcat(buf, itoa_m0(tmp, 10, 2));
	tmp = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC));
	strcat(buf, itoa_m0(tmp, 10, 2));

	return buf;
}
