// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimFlightEstimator_hpp
#define msr_airlib_AirSimFlightEstimator_hpp

#include "firmware/interfaces/CommonStructs.hpp"
#include "AirSimPlaneFlightCommon.hpp"
#include "physics/Kinematics.hpp"
#include "physics/Environment.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {


class AirSimPlaneFlightEstimator : public plane_flight::IStateEstimator {
public:

    virtual ~AirSimPlaneFlightEstimator() {}
    
    //for now we don't do any state estimation and use ground truth (i.e. assume perfect sensors)
    void setGroundTruthKinematics(const Kinematics::State* kinematics, const Environment* environment)
    {
        kinematics_ = kinematics;
        environment_ = environment;
    }

    virtual plane_flight::Axis3r getAngles() const override
    {
        plane_flight::Axis3r angles;
        VectorMath::toEulerianAngle(kinematics_->pose.orientation,
            angles.pitch(), angles.roll(), angles.yaw());

        //Utils::log(Utils::stringf("Ang Est:\t(%f, %f, %f)", angles.pitch(), angles.roll(), angles.yaw()));

        return angles;
    }

    virtual plane_flight::Axis3r getAngularVelocity() const override
    {
        const auto& anguler = kinematics_->twist.angular;

        plane_flight::Axis3r conv;
        conv.x() = anguler.x(); conv.y() = anguler.y(); conv.z() = anguler.z();

        return conv;
    }

    virtual plane_flight::Axis3r getPosition() const override
    {
        return AirSimPlaneFlightCommon::toAxis3r(kinematics_->pose.position);
    }

    virtual plane_flight::Axis3r transformToBodyFrame(const plane_flight::Axis3r& world_frame_val) const override
    {
        const Vector3r& vec = AirSimPlaneFlightCommon::toVector3r(world_frame_val);
        const Vector3r& trans = VectorMath::transformToBodyFrame(vec, kinematics_->pose.orientation);
        return AirSimPlaneFlightCommon::toAxis3r(trans);
    }

    virtual plane_flight::Axis3r getLinearVelocity() const override
    {
        return AirSimPlaneFlightCommon::toAxis3r(kinematics_->twist.linear);
    }

    virtual plane_flight::Axis4r getOrientation() const override
    {
        return AirSimPlaneFlightCommon::toAxis4r(kinematics_->pose.orientation);
    }

    virtual plane_flight::GeoPoint getGeoPoint() const override
    {
        return AirSimPlaneFlightCommon::toFlightGeoPoint(environment_->getState().geo_point);
    }

    virtual plane_flight::GeoPoint getHomeGeoPoint() const override
    {
        return AirSimPlaneFlightCommon::toFlightGeoPoint(environment_->getHomeGeoPoint());
    }
	
    virtual plane_flight::KinematicsState getKinematicsEstimated() const override
    {
        plane_flight::KinematicsState state;
        state.position = getPosition();
        state.orientation = getOrientation();
        state.linear_velocity = getLinearVelocity();
        state.angular_velocity = getAngularVelocity();
        state.linear_acceleration = AirSimPlaneFlightCommon::toAxis3r(kinematics_->accelerations.linear);
        state.angular_acceleration = AirSimPlaneFlightCommon::toAxis3r(kinematics_->accelerations.angular);
        
        return state;
    }


private:
    const Kinematics::State* kinematics_;
    const Environment* environment_;
};


}} //namespace
#endif
