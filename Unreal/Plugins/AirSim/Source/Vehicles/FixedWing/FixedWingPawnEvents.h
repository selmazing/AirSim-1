#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "PawnEvents.h"
#include "common/Common.hpp"
#include "common/common_utils/Signal.hpp"


class FixedWingPawnEvents : public PawnEvents {
public: //types
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Vector3r Vector3r;
    struct FixedWingControlsInfo {
        real_T control_deflection = 0; // angle of control [deg] +ve is down
        real_T control_speed = 0; // angular control rotation rate [deg/s] +ve is down
        real_T control_filtered = 0; // commanded control deflection [deg] +ve is down
        Vector3r rotation_plane = {0, 0, 0}; // the vector along which the control rotates what about ailerons???
        // testing git
    };

    typedef common_utils::Signal<const std::vector<FixedWingControlsInfo>&> ActuatorsSignal;

public:
    ActuatorsSignal& getActuatorSignal();

private:
    ActuatorsSignal actuator_signal_;
};