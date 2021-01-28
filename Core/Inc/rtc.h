#ifndef _MRTC_H_
#define _MRTC_H_

#include <stdint.h>

void RTC_change_time(uint8_t new_hh, uint8_t new_mm, uint8_t new_ss);
void RTC_change_date(uint8_t new_yy, uint8_t new_mm, uint8_t new_dd, uint8_t new_dow);
uint8_t RTC_set_time_from_st(char *st_time);
uint8_t RTS_set_date_from_st(char *st_date);
char * RTC_get_time_to_str(void);

#endif