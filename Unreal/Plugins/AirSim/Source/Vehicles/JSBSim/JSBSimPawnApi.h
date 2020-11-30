#pragma once

#include "vehicles/jsbsim/api/JSBSimApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "JSBSimPawn.h"

class JSBSimPawnApi {
public:
	typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

	JSBSimPawnApi(AJSBSimPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics,
		msr::airlib::JSBSimApiBase* vehicle_api);

	void updateMovement(const msr::airlib::JSBSimApiBase::JSBSimControls& controls);

	msr::airlib::JSBSimApiBase::JSBSimState getJSBSimState() const;

	void reset();
	void update();

	virtual ~JSBSimPawnApi();

private:
	msr::airlib::JSBSimApiBase::JSBSimControls last_controls_;
	msr::airlib::JSBSimApiBase::JSBSimState last_state_;
	AJSBSimPawn* pawn_;
	const msr::airlib::Kinematics::State* pawn_kinematics_;
	msr::airlib::JSBSimApiBase* vehicle_api_;
	msr::airlib::JSBSimPhysicsEngine* jsbsim_physics_;
};