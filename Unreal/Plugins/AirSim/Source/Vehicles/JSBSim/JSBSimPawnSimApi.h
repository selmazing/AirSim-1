#pragma once

#include "CoreMinimal.h"
#include "PawnSimApi.h"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "vehicles/jsbsim/api/JSBSimApiBase.hpp"
#include "JSBSimPawnEvents.h"
#include <future>
#include "physics/JSBSimPhysicsEngine.hpp"

class JSBSimPawnSimApi : public PawnSimApi
{
public:
	typedef msr::airlib::real_T real_T;
	typedef msr::airlib::Utils Utils;
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
	virtual void reportState(StateReporter& reporter) override;

	virtual void setPose(const Pose& pose, bool ignore_collision) override;
	// virtual Pose getPose() const override; // get the pose from the simulation
	//virtual void updateApiState(); // updates the JSBSimState Struct in JSBSimApiBase 
	virtual Kinematics::State getState() const;
	virtual void pawnTick(float dt) override;

	// get the JSBSim API implementation
	msr::airlib::JSBSimApiBase* getVehicleApi() const
	{
		return vehicle_api_.get();
	}

	// get the vehicle sim api base
	virtual msr::airlib::VehicleApiBase* getVehicleApiBase() const override
	{
		return vehicle_api_.get();
	}

	// get the JSBSim physics object
	msr::airlib::JSBSimPhysicsEngine* getJSBSimPhysics() const
	{
		return jsbsim_.get();
	}

private:
	//api and params needed to be setup & updated
	std::unique_ptr<msr::airlib::JSBSimApiBase> vehicle_api_;
	// std::unique_ptr<JSBSimPawnSimApi> pawn_api_;
	std::unique_ptr<JSBSimPhysicsEngine> jsbsim_;
	
	//show info on collision response from UE4
	CollisionResponse collision_response;

	// when pose needs to be set from a non-physics thread it should be set as pending
	bool pending_pose_collisions_;
	enum class PendingPoseStatus {
		NonePending, RenderStatePending, RenderPending
	} pending_pose_status_;
	Pose pending_phys_pose_; //force new pose through API
	
	JSBSimPawnEvents* pawn_events_;
	Params params_;
	Pose last_phys_pose_; // to obtain trace lines from the API
	std::vector<std::string> vehicle_api_messages_;

	//reset must happen while World is locked so its async task initiated from API thread
	bool reset_pending_;
	bool did_reset_;
	std::packaged_task<void()> reset_task_;
	
};
