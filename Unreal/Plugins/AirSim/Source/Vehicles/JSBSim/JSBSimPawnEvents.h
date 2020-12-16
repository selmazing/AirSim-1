#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "PawnEvents.h"
#include "common/Common.hpp"
#include "common/common_utils/Signal.hpp"

/* This just creates JSBSimPawnEvents instance of PawnEvents for use in JSBSimPawn.
 * The class used in JSBSSimPawn should probably just be instanced there. 
 */

class JSBSimPawnEvents : public PawnEvents 
{
public:
	typedef msr::airlib::real_T real_T;	
};

