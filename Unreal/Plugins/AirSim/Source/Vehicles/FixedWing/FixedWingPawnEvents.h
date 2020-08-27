#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "PawnEvents.h"
#include "common/Common.hpp"
#include "common/common_utils/Signal.hpp"

/* This started of as a generic control constructor, but given they are all quite different in axis and implementation it became easier
to define a control class for each control*/
class FixedWingPawnEvents : public PawnEvents {
public: //types
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Vector3r Vector3r;
    struct FixedWingControlInfo
	{
        real_T control_deflection = 0; // angle of control [deg] +ve is down      
        real_T control_speed = 0; // angular control rotation rate [deg/s] +ve is down
        real_T control_command = 0; // commanded control deflection [deg] +ve is down
    	//Vector3r rotation_plane = {0, 0, 0}; // the vector along which the control rotates what about ailerons???
    };

    typedef common_utils::Signal<const std::vector<FixedWingControlInfo>&> ControlSignal; //can we set the signal to a constant rather than a vector?
   

public:
    ControlSignal& getControlSignal();

private:
    ControlSignal control_signal_;
};