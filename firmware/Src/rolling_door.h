/*
 * rolling_doors.h
 *
 *  Created on: 04 Aug 2019
 *      Author: jcera
 */

#ifndef SRC_ROLLING_DOOR_H_
#define SRC_ROLLING_DOOR_H_
#include <stdint.h>
#include "garage_light.h"

class RollingDoor
{
public:
	enum eDoorState
	{
		UNKNOWN,
		CLOSED,
		MOVING,
		OPEN
	};

private:
	eDoorState mDoorState;
	const char *mName;
	void (*mSamplePins)(uint8_t *states);

	eDoorState getState(uint8_t states);
	const char *getStateName(RollingDoor::eDoorState state);

	GarageLight *mLight;

public:
	RollingDoor(const char *name, void (*samplePins)(uint8_t *states));
	virtual ~RollingDoor();

	bool run();

	eDoorState getState(){ return mDoorState; }

	void setLigth(GarageLight *light){ mLight = light; }
};

#endif /* SRC_ROLLING_DOOR_H_ */
