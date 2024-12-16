/*
 * rtc.c
 *
 *  Created on: Dec 15, 2024
 *      Author: sajanduwal
 */

#include "main.h"
#include "rtc.h"

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

RTC_TimeTypeDef gTime;
RTC_DateTypeDef gDate;

void setTime(uint8_t year, uint8_t month, uint8_t weekDay, uint8_t hour,
		uint8_t min, uint8_t sec) {

	sTime.Hours = hour;
	sTime.Minutes = min;
	sTime.Seconds = sec;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
		Error_Handler();

	sDate.Month = month;
	sDate.Year = year;
	sDate.WeekDay = weekDay;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
		Error_Handler();

	//HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);

}

void getTime() {
	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

}
