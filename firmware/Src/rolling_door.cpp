/*
 * rolling_doors.cpp
 *
 *  Created on: 04 Aug 2019
 *      Author: jcera
 */
#include <stdio.h>

#include "rolling_door.h"

RollingDoor::RollingDoor(const char *name, void (*samplePins)(uint8_t *states)) :
	mDoorState(UNKNOWN),
	mName(name),
	mSamplePins(samplePins),
	mLight(0)
{
}

RollingDoor::~RollingDoor()
{
}


RollingDoor::eDoorState RollingDoor::getState(uint8_t states)
{
	switch(states)
	{
	case 0x01:
		return OPEN;
	case 0x02:
		return CLOSED;
	default:
		return MOVING;
	}
}

const char *RollingDoor::getStateName(RollingDoor::eDoorState state)
{
	switch(state)
	{
	case OPEN:
		return "OPEN";
	case CLOSED:
		return "CLOSED";
	case MOVING:
		return "MOVING";
	default:
		return "UNKNOWN";
	}
}

bool RollingDoor::run()
{
	if(mSamplePins)
	{
		uint8_t states;
		mSamplePins(&states);
		//printf("%s %02X\n", mName, states);

		eDoorState state = getState(states);
		if(mDoorState != state)
		{
			mDoorState = state;
			printf("%s - %s\n", mName, getStateName(state));

			if(mLight)
			{
				if((state == MOVING) || (state == OPEN))
					mLight->setDoor(true);
				else
					mLight->setDoor(false);
			}

			return true;
		}
	}

	return false;
}

