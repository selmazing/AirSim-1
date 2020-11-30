#include "JSBSimPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "JSBSimPawnApi.h"
#include <exception>

using namespace msr::airlib; // scoping across the whole namespace like this could cause confusion

JSBSimPawnSimApi::JSBSimPawnSimApi(const Params& params)
	: PawnSimApi(params),
	pawn_events_(static_cast<JSBSimPawnEvents*>(params.pawn_events))
{
	Pose pose = getPose();
	float pitch, roll, yaw;
	VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
	pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
	setPose(pose, false);
}

void JSBSimPawnSimApi::initialize()
{
	PawnSimApi::initialize();

	//create the vehicle API
	std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
	/*Insert code here for vehicle_params_ and vehicle_api for the JSBSim API*/

	//setup physics vehicle
	/*Insert code to setup a JSBSim physics body, just call JSBSim init method?*/
}

void JSBSimPawnSimApi::createVehicleApi(AJSBSimPawn* pawn)
{
	std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
}

void JSBSimPawnSimApi::updateRenderedState(float dt)
{
	/* Implement code to update the aircraft's state, should call the JSBSim interface here
	 * Multirotor implementation deals with update of collision here too. 
	 */
}

void JSBSimPawnSimApi::updateRendering(float dt)
{
	PawnSimApi::updateRendering(dt);
	
	/* Implement code to update the aircraft's rendered state
	 * Multirotor implementation deals with the graphic collision render hand-off here too
	 */
}

void JSBSimPawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
	pending_physics_pose_ = pose;
	pending_pose_collisions_ = ignore_collision;
	pending_pose_status_ = PendingPoseStatus::RenderStatePending;
}

/* Start: UpdatableState implementation */
void JSBSimPawnSimApi::resetImplementation()
{
	PawnSimApi::resetImplementation();

	vehicle_api_->reset();
	jsbsim_physics_body_->reset();
	vehicle_api_messages_.clear();
}

// This is the high-frequency physics update, the aircraft is updated at the same frame rate as the render
void JSBSimPawnSimApi::update()
{
	//update environment
	PawnSimApi::update();

	
	jsbsim_physics_body_->update();

}

void JSBSimPawnSimApi::reportState(StateReporter& reporter)
{
	PawnSimApi::reportState(reporter);
	jsbsim_physics_body_->reportState(reporter);
}

JSBSimPawnSimApi::UpdatableObject* JSBSimPawnSimApi::getPhysicsBody()
{
	return jsbsim_physics_body_->getPhysicsBody();
	
}
//*** End: UpdatableState implementation ***//


