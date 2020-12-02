#include "JSBSimPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "AirLib/include/physics/JSBSimPhysicsEngine.hpp"
#include "JSBSimPawnApi.h"
#include <exception>

#include "vehicles/jsbsim/JSBSimApiFactory.hpp"

using namespace msr::airlib; // scoping across the whole namespace like this could cause confusion

JSBSimPawnSimApi::JSBSimPawnSimApi(const Params& params)
	: PawnSimApi(params),
	pawn_events_(static_cast<AJSBSimPawnEvents*>(params.pawn_events))
{
	Pose pose = getPose();
	float pitch, roll, yaw;
	VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
	pose.orientation = VectorMath::toQuaternion(0, 0, yaw); // start the aircraft at a level attitude
	setPose(pose, false);
}

//TODO: Find way to sync jsbsim_ delta_t and AirSim dt provided in API
//TODO: Setup Paths to JSBSimFGFDMExec model, this should come in the model paths for JSBSim and done in API
void JSBSimPawnSimApi::initialize()
{
	PawnSimApi::initialize();

	//create the vehicle API
	std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
	/*Insert code here for vehicle_params_ and vehicle_api for the JSBSim API*/
	// vehicle_api_ = JSBSimApiFactory::createApi(getVehicleSetting(), sensor_factory, );
	
	//setup JSBSim
	/*Insert code to setup a JSBSim engine, just call JSBSim init method?*/
	// jsbsim_->setDeltaT(); // sets the value of DeltaT not sure where to get the value from though?
	// jsbsim_->setPaths(); // sets the paths to the model used for JSBSim 
	jsbsim_->initialize(); // initialize JSBSim


	// Initialize private variables
	last_phys_pose_ = pending_phys_pose_ = Pose::nanPose();
	pending_pose_status_ = PendingPoseStatus::NonePending;
	reset_pending_ = false;
	did_reset_ = false;
}

void JSBSimPawnSimApi::pawnTick(float dt)
{
	unused(dt);
	// Handled in JSBSim with set(dt) and 'ticked' by run command
}

void JSBSimPawnSimApi::updateRenderedState(float dt)
{
	/* Implement code to update the aircraft's state, Multirotor deals with asynco pending here I believe. 
	 */
	// if reset is pending the do it first, no need to do other things until the next tick
	if(reset_pending_)
	{
		reset_task_();
		did_reset_ = true;
		return;
	}

	const CollisionInfo& collision_info = getCollisionInfo();
	jsbsim_->setCollisionInfo(collision_info);

	if(pending_pose_status_ == PendingPoseStatus::RenderStatePending)
	{
		// jsbsim_->setPose(pending_phys_pose_); // this is what multirotor does don't know if I want to directly set JSBSim pose
		pending_pose_status_ = PendingPoseStatus::RenderPending;
	}

	last_phys_pose_ = jsbsim_->getPose(); // getPose from JSBSim
	// collision_response = jsbsim_->getCollisionResponseInfo(); // used to get the response of the aircraft colliding with the ground
	vehicle_api_->getStatusMessages(vehicle_api_messages_);
}

/* void JSBSimPawnSimApi::updateApiState()
{
	JSBSimApiBase::JSBSimState state;
	state.collision = getCollisionInfo;
	state.kinematics_estimated = jsbsim_->getKinematicState();
	state.timestamp = static_cast<uint64_t>(jsbsim_->getTime());
	vehicle_api_->getJSBSimState() = state;
	
} */

void JSBSimPawnSimApi::updateRendering(float dt)
{
	PawnSimApi::updateRendering(dt);
	
	/* Implement code to update the aircraft's rendered state
	 * Multirotor implementation deals with the graphic collision render hand-off here too
	 */
	if(reset_pending_)
	{
		//continue to wait for reset
		if(!did_reset_)
		{
			return;
		}
		else
		{
			reset_pending_ = false;
			did_reset_ = false;
			return;
		}
	}

	if (!VectorMath::hasNan(last_phys_pose_))
	{
		if (pending_pose_status_ == PendingPoseStatus::RenderPending)
		{
			PawnSimApi::setPose(last_phys_pose_, pending_pose_collisions_);
			pending_pose_status_ = PendingPoseStatus::NonePending;
		} else
		{
			PawnSimApi::setPose(last_phys_pose_, false);
		}
	}

	for (auto i = 0; i < vehicle_api_messages_.size(); ++i)
	{
		UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
	}

	try
	{
		vehicle_api_->sendTelemetry(dt);
	}
	catch (std::exception &e)
	{
		UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
	}
	
}

void JSBSimPawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
	pending_phys_pose_ = pose;
	pending_pose_collisions_ = ignore_collision;
	pending_pose_status_ = PendingPoseStatus::RenderStatePending;
}

Pose JSBSimPawnSimApi::getPose() const
{
	return pending_phys_pose_;
}

Kinematics::State JSBSimPawnSimApi::getState() const
{
	return jsbsim_->getKinematicState();
}


/* Start: UpdatableState implementation */
void JSBSimPawnSimApi::resetImplementation()
{
	PawnSimApi::resetImplementation();

	vehicle_api_->reset();
	jsbsim_->reset();
	vehicle_api_messages_.clear();
}

// This is the high-frequency physics update, the aircraft is updated at the same frame rate as the render
void JSBSimPawnSimApi::update()
{
	//update environment
	PawnSimApi::update();
	// update JSBSimPhysicsEngine, i.e. run one time instance
	jsbsim_->update();
}

void JSBSimPawnSimApi::reportState(StateReporter& reporter)
{
	PawnSimApi::reportState(reporter);
	jsbsim_->reportState(reporter);
}
//*** End: UpdatableState implementation ***//


