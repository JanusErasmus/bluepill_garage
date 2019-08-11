#ifndef SRC_GARAGE_LIGHT_H_
#define SRC_GARAGE_LIGHT_H_

#include "stm32f1xx_hal.h"

class GarageLight
{
	bool mLightState;
	bool mRequestState;
	uint32_t mSwitchTime;
	void (*mSetLight)(bool);
public:
	GarageLight(void (*set_cb)(bool));
	virtual ~GarageLight();

	void run();
	void setDoor(bool open);
};

#endif /* SRC_GARAGE_LIGHT_H_ */
