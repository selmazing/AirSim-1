#include "JSBSimPawnApi.h"
#include "AirBlueprintLib.h"

JSBSimPawnApi::JSBSimPawnApi(AJSBSimPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics,
	msr::airlib::JSBSimApiBase* vehicle_api) : pawn_(pawn), pawn_kinematics_(pawn_kinematics), vehicle_api_(vehicle_api)
{
}

void JSBSimPawnApi::updateMovement(const msr::airlib::JSBSimApiBase::JSBSimControls& controls)
{
	last_controls_ = controls;

	
}

msr::airlib::JSBSimApiBase::JSBSimState JSBSimPawnApi::getJSBSimState() const
{
	msr::airlib::JSBSimApiBase::JSBSimState state(
		// collision
		// state
		// timestamp
	);
	return state;
}

void JSBSimPawnApi::reset()
{
	vehicle_api_->reset();
	
	
}