#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "PawnEvents.h"
#include "common/Common.hpp"
#include "common/common_utils/Signal.hpp"

/* This started of as a generic control constructor, but given they are all quite diffrent in axis and implementation it became easier
to define a control class for each control*/
class FixedWingPawnEvents : public PawnEvents {
public: //types
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Vector3r Vector3r;
    struct FixedWingElevatorInfo
	{
        real_T elevator_deflection = 0; // angle of control [deg] +ve is down      
        real_T elevator_speed = 0; // angular control rotation rate [deg/s] +ve is down
        real_T elevator_command = 0; // commanded control deflection [deg] +ve is down
    	//Vector3r rotation_plane = {0, 0, 0}; // the vector along which the control rotates what about ailerons???
    };

    struct FixedWingAileronInfo
    {
        real_T aileron_deflection = 0; // angle of control [deg] +ve is down right
        real_T aileron_speed = 0; // angular control rotation rate [deg/s] +ve is down right
        real_T aileron_command = 0; // commanded control deflection [deg] +ve is down right
    };

	struct FixedWingRudderInfo
	{
        real_T rudder_deflection = 0; // angle of control [deg] +ve is right
        real_T rudder_speed = 0; // angular control rotation rate [deg/s] +ve is right
        real_T rudder_command = 0; // commanded control deflection [deg] +ve is right
	};

    typedef common_utils::Signal<const std::vector<FixedWingElevatorInfo>&> ElevatorSignal; //can we set the signal to a constant rather than a vector?
    typedef common_utils::Signal<const std::vector<FixedWingAileronInfo>&> AileronSignal;
    typedef common_utils::Signal<const std::vector<FixedWingRudderInfo>&> RudderSignal;

public:
    ElevatorSignal& getElevatorSignal();
    AileronSignal& getAileronSignal();
    RudderSignal& getRudderSignal();

private:
    ElevatorSignal elevator_signal_;
    AileronSignal aileron_signal_;
	RudderSignal rudder_signal_;
};