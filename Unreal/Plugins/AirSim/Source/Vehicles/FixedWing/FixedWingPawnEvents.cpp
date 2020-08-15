#include "FixedWingPawnEvents.h"

FixedWingPawnEvents::ElevatorSignal& FixedWingPawnEvents::getElevatorSignal()
{
    return elevator_signal_;
}

FixedWingPawnEvents::AileronSignal& FixedWingPawnEvents::getAileronSignal()
{
	return aileron_signal_;
}

FixedWingPawnEvents::RudderSignal& FixedWingPawnEvents::getRudderSignal()
{
	return rudder_signal_;
}

