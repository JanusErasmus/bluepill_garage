/*
 * garage_light.cpp
 *
 *  Created on: 08 Aug 2019
 *      Author: jcera
 */

#include "garage_light.h"

#include "sunrise.h"

extern
RTC_HandleTypeDef hrtc;

bool isDay()
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	if(sunrise_is_day(sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes) == 1)
		return true;

	return false;
}

GarageLight::GarageLight(void (*set_cb)(bool)) : mSetLight(set_cb)
{
	mLightState = false;
	mSwitchTime = 0;
	mRequestState = false;
}

GarageLight::~GarageLight()
{
}

void GarageLight::run()
{
	uint32_t onTime = HAL_GetTick() - mSwitchTime;

	if(mLightState != mRequestState)
	{
		if(isDay())
		{
			printf("It is day, switching light off\n");
			mSetLight(false);
			mLightState = false;
			mRequestState = mLightState;
		}

		//only when light has been on for 1.5 min, turn it off
		if(onTime > 90000)
		{
			printf("Light has been on for longer than %d ms\n", (int)onTime);
			mSetLight(false);
			mLightState = false;
			mRequestState = mLightState;
		}
	}

	//only when light has been on for 5 min, turn it off anyways
	if(mLightState)
	{
		if(onTime > 300000)
		{
			printf("Light has been on for too long (%d ms)\n", (int)onTime);
			mSetLight(false);
			mLightState = false;
			mRequestState = mLightState;
		}
	}
}

void GarageLight::setDoor(bool open)
{
	mRequestState = open;

	//when door opens, switch light on
	if(open && !mLightState)
	{
		if(isDay())
		{
			printf("It is day, NOT switching light on\n");
			mSetLight(false);
			mLightState = false;
		}
		else
		{
			mSwitchTime = HAL_GetTick();
			printf("Switch light on: %d\n", (int)mSwitchTime);

			mSetLight(true);
			mLightState = true;
		}
	}
	else
	{
		uint32_t onTime = HAL_GetTick() - mSwitchTime;

		//only when light has been on for 1.5 min, turn it off
		if(onTime > 90000)
		{
			printf("Light has been on for %d ms\n", (int)onTime);
			mSetLight(false);
			mLightState = false;
		}
	}

}
