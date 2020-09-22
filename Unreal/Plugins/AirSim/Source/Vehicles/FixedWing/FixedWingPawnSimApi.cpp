#include "FixedWingPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "vehicles/fixedwing/FixedWingParamsFactory.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
#include <exception>

using namespace msr::airlib;

FixedWingPawnSimApi::FixedWingPawnSimApi(const Params& params)
    : PawnSimApi(params),
      pawn_events_(static_cast<FixedWingPawnEvents*>(params.pawn_events))
{
    //reset roll & pitch of vehicle as multirotors required to be on plain surface at start <-- equivalent fixedwing
    Pose pose = getPose();
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
    pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
    setPose(pose, false);
}

void FixedWingPawnSimApi::initialize()
{
    PawnSimApi::initialize();

    //create vehicle API
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    vehicle_params_ = FixedWingParamsFactory::createConfig(getVehicleSetting(), sensor_factory);
    vehicle_api_ = vehicle_params_->createFixedWingApi();
    //setup physics vehicle
    fixedwing_physics_body_ = std::unique_ptr<FixedWing>(new FixedWingPhysicsBody(vehicle_params_.get(), vehicle_api_.get(),
        getKinematics(), getEnvironment()));
    control_count_ = fixedwing_physics_body_->controlCount();
    control_info_.assign(control_count_, FixedWingControlInfo());

    vehicle_api_->setSimulatedGroundTruth(getGroundTruthKinematics(), getGroundTruthEnvironment());

    //initialize private vars
    last_phys_pose_ = pending_phys_pose_ = Pose::nanPose();
    pending_pose_status_ = PendingPoseStatus::NonePending;
    reset_pending_ = false;
    did_reset_ = false;
}

void FixedWingPawnSimApi::pawnTick(float dt)
{
    unused(dt);
    //calls to update* are handled by physics engine and in SimModeWorldBase
}

void FixedWingPawnSimApi::updateRenderedState(float dt)
{
    //Utils::log("------Render tick-------");

    //if reset is pending then do it first, no need to do other things until next tick
    if (reset_pending_) {
        reset_task_();
        did_reset_ = true;
        return;
    }

    //move collision info from rendering engine to vehicle
    const CollisionInfo& collision_info = getCollisionInfo();
    fixedwing_physics_body_->setCollisionInfo(collision_info);

    if (pending_pose_status_ == PendingPoseStatus::RenderStatePending) {
        fixedwing_physics_body_->setPose(pending_phys_pose_);
        pending_pose_status_ = PendingPoseStatus::RenderPending;
    }
        
    last_phys_pose_ = fixedwing_physics_body_->getPose();
    
    collision_response = fixedwing_physics_body_->getCollisionResponseInfo();

    //update control poses
    for (unsigned int i = 0; i < control_count_; i++)
    {
        const auto& control_output = fixedwing_physics_body_->getControlSurfaceOutput(i);
        FixedWingControlInfo* info = &control_info_[i];
        info->control_speed = control_output.control_speed;
        info->control_command = control_output.control_signal_filtered;
        info->control_deflection = control_output.control_signal_input;
    }
		
    vehicle_api_->getStatusMessages(vehicle_api_messages_);

    if (getRemoteControlID() >= 0)
        vehicle_api_->setRCData(getRCData());
}

void FixedWingPawnSimApi::updateRendering(float dt)
{
    //if we did reset then don't worry about synchronizing states for this tick
    if (reset_pending_) {
        // Continue to wait for reset
        if (!did_reset_) {
            return;
        }
        else {
            reset_pending_ = false;
            did_reset_ = false;
            return;
        }
    }

    if (!VectorMath::hasNan(last_phys_pose_)) {
        if (pending_pose_status_ ==  PendingPoseStatus::RenderPending) {
            PawnSimApi::setPose(last_phys_pose_, pending_pose_collisions_);
            pending_pose_status_ = PendingPoseStatus::NonePending;
        }
        else
            PawnSimApi::setPose(last_phys_pose_, false);
    }

    //UAirBlueprintLib::LogMessage(TEXT("Collision (raw) Count:"), FString::FromInt(collision_response.collision_count_raw), LogDebugLevel::Unimportant);
    UAirBlueprintLib::LogMessage(TEXT("Collision Count:"), 
        FString::FromInt(collision_response.collision_count_non_resting), LogDebugLevel::Informational);

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception &e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }

    pawn_events_->getControlSignal().emit(control_info_);
}

void FixedWingPawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
    pending_phys_pose_ = pose;
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderStatePending;
}

//*** Start: UpdatableState implementation ***//
void FixedWingPawnSimApi::resetImplementation()
{
    PawnSimApi::resetImplementation();

    vehicle_api_->reset();
    fixedwing_physics_body_->reset();
    vehicle_api_messages_.clear();
}

//this is high frequency physics tick, flier gets ticked at rendering frame rate
void FixedWingPawnSimApi::update()
{
    //environment update for current position
    PawnSimApi::update();

    //update forces on vertices
    fixedwing_physics_body_->update();

    //update to controller must be done after kinematics have been updated by physics engine
}

void FixedWingPawnSimApi::reportState(StateReporter& reporter)
{
    PawnSimApi::reportState(reporter);

    fixedwing_physics_body_->reportState(reporter);
}

FixedWingPawnSimApi::UpdatableObject* FixedWingPawnSimApi::getPhysicsBody()
{
    return fixedwing_physics_body_->getPhysicsBody();
}
//*** End: UpdatableState implementation ***//

