#pragma once

#include "CoreMinimal.h"

#include "PawnSimApi.h"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "vehicles/jsbsim/api/JSBSimApiBase.hpp"
// #include "AirLib/include/vehicles/jsbsim/JSBSimPhysicsBody.hpp"
#include "JSBSimPawnEvents.h"

class JSBSimPawnSimApi : public PawnSimApi
{
public:
	typedef msr::airlib::real_T real_T;
	typedef msr::airlib::Utils Utils;
	// typedef msr::airlib::JSBSimPhysicsBody JSBSim;
	typedef msr::airlib::StateReporter StateReporter;
	typedef msr::airlib::UpdatableObject UpdatableObject;
	typedef msr::airlib::Pose Pose;

public:
	virtual void initialize() override;
	virtual ~JSBSimPawnSimApi() = default;

	//VehicleSimApiBase interface, implements game interface to update pawn
	JSBSimPawnSimApi(const Params& params);
	virtual void update() override;
	virtual void updateRenderedState(float dt) override;
	virtual void updateRendering(float dt) override;

	//PhysicsBody interface, wrapper for JSBSim physics
	virtual void resetImplementation() override;
	// virtual void reportState(StateReporter& reporter) override;
	// UpdatableObject* getPhysicsBody() override;

	// virtual void setPose(const Pose& pose, bool ignore_collision) override;
	// virtual void pawnTick(float dt) override;

private:
	//show info on collision response from UE4
	msr::airlib::CollisionResponse collision_response;
	// std::unique_ptr<JSBSim> jsbsim_physics_body_;

	JSBSimPawnEvents* pawn_events_;

	Params para
	Pose last_phys_pose; // to obtain trace lines from the API
	std::vector<std::string> vehicle_api_messages_;
	std::unique_ptr<msr::airlib::JSBSimApiBase> vehicle_api_;
	std::unique_ptr<JSBSimPawnSimApi> pawn_api_;
	
};