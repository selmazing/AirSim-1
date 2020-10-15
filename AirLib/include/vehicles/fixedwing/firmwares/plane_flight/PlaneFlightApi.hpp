// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SimplePlaneFlightController_hpp
#define msr_airlib_SimplePlaneFlightController_hpp

#include "vehicles/fixedwing/api/FixedWingApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/fixedwing/FixedWingParams.hpp"
#include "common/Common.hpp"
#include "firmware/Firmware.hpp"
#include "AirSimPlaneFlightBoard.hpp"
#include "AirSimPlaneFlightCommLink.hpp"
#include "AirSimPlaneFlightEstimator.hpp"
#include "AirSimPlaneFlightCommon.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/AirSimSettings.hpp"

//TODO: we need to protect contention between physics thread and API server thread

namespace msr { namespace airlib {

class PlaneFlightApi : public FixedWingApiBase {

public:
    PlaneFlightApi(const FixedWingParams* vehicle_params, const AirSimSettings::VehicleSetting* vehicle_setting)
        : vehicle_params_(vehicle_params)
    {
        readSettings(*vehicle_setting);

        //TODO: set below properly for better high speed safety
        safety_params_.vel_to_breaking_dist = safety_params_.min_breaking_dist = 0;

        //create sim implementations of board and commlink
        board_.reset(new AirSimPlaneFlightBoard(&params_));
        comm_link_.reset(new AirSimPlaneFlightCommLink());
        estimator_.reset(new AirSimPlaneFlightEstimator());

        //create firmware
        firmware_.reset(new plane_flight::Firmware(&params_, board_.get(), comm_link_.get(), estimator_.get()));
    }


public: //VehicleApiBase implementation
    virtual void resetImplementation() override
    {
        FixedWingApiBase::resetImplementation();

        firmware_->reset();
    }
    virtual void update() override
    {
        FixedWingApiBase::update();
        //update controller which will update actuator control signal
        firmware_->update();
    }
    virtual bool isApiControlEnabled() const override
    {
        return firmware_->offboardApi().hasApiControl();
    }
    virtual void enableApiControl(bool is_enabled) override
    {
        if (is_enabled) {
            //comm_link should print message so no extra handling for errors
            std::string message;
            firmware_->offboardApi().requestApiControl(message);
        }
        else
            firmware_->offboardApi().releaseApiControl();
    }
    virtual bool armDisarm(bool arm) override
    {
        std::string message;
        if (arm)
            return firmware_->offboardApi().arm(message);
        else
            return firmware_->offboardApi().disarm(message);
    }
    virtual GeoPoint getHomeGeoPoint() const override
    {
        Utils::log("Not Implemented: getHomeGeoPoint", Utils::kLogLevelInfo);
        return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
    }
    virtual void getStatusMessages(std::vector<std::string>& messages) override
    {
        comm_link_->getStatusMessages(messages);
    }

    virtual const SensorCollection& getSensors() const override
    {
        return vehicle_params_->getSensors();
    }

public: //FixedWingApiBase implementation

    virtual real_T getActuation(unsigned actuator_index) const override
    {
        switch (actuator_index)
        {
        case 0: return aileron_deflection_;

        case 1: return elevator_deflection_;

        case 2: return tla_deflection_;

        case 3: return rudder_deflection_;

        default: return 0.0f;
        }
    }

    virtual real_T getElevatorSignal() const
    {
        return elevator_deflection_;
    }

    virtual real_T getAileronSignal() const
    {
        return aileron_deflection_;
    }

    virtual real_T getRudderSignal() const
    {
        return rudder_deflection_;
    }

    virtual real_T getTLASignal() const
    {
        return tla_deflection_;
    }
	
    virtual void moveByRC(const RCData& rc_data) override
    {
        setRCData(rc_data);
    }
    virtual void setSimulatedGroundTruth(const Kinematics::State* kinematics, const Environment* environment) override
    {
        board_->setGroundTruthKinematics(kinematics);
        estimator_->setGroundTruthKinematics(kinematics, environment);
    }
    virtual bool setRCData(const RCData& rc_data) override
    {
        last_rcData_ = rc_data;
        if (rc_data.is_valid) {
            board_->setIsRcConnected(true);
            board_->setInputChannel(0, rc_data.roll); //X
            board_->setInputChannel(1, rc_data.yaw); //Y
            board_->setInputChannel(2, rc_data.throttle); //F
            board_->setInputChannel(3, -rc_data.pitch); //Z
            board_->setInputChannel(4, static_cast<float>(rc_data.getSwitch(0))); //angle rate or level
            board_->setInputChannel(5, static_cast<float>(rc_data.getSwitch(1))); //Allow API control
            board_->setInputChannel(6, static_cast<float>(rc_data.getSwitch(2)));
            board_->setInputChannel(7, static_cast<float>(rc_data.getSwitch(3)));
            board_->setInputChannel(8, static_cast<float>(rc_data.getSwitch(4)));
            board_->setInputChannel(9, static_cast<float>(rc_data.getSwitch(5)));
            board_->setInputChannel(10, static_cast<float>(rc_data.getSwitch(6)));
            board_->setInputChannel(11, static_cast<float>(rc_data.getSwitch(7)));
        }
        else { //else we don't have RC data
            board_->setIsRcConnected(false);
        }

        return true;
    }

protected:
    virtual Kinematics::State getKinematicsEstimated() const override
    {
        return AirSimPlaneFlightCommon::toKinematicsState3r(firmware_->offboardApi().
            getStateEstimator().getKinematicsEstimated());
    }

    virtual Vector3r getPosition() const override
    {
        const auto& val = firmware_->offboardApi().getStateEstimator().getPosition();
        return AirSimPlaneFlightCommon::toVector3r(val);
    }

    virtual Vector3r getVelocity() const override
    {
        const auto& val = firmware_->offboardApi().getStateEstimator().getLinearVelocity();
        return AirSimPlaneFlightCommon::toVector3r(val);
    }

    virtual Quaternionr getOrientation() const override
    {
        const auto& val = firmware_->offboardApi().getStateEstimator().getOrientation();
        return AirSimPlaneFlightCommon::toQuaternion(val);
    }

    virtual LandedState getLandedState() const override
    {
        return firmware_->offboardApi().getLandedState() ? LandedState::Landed : LandedState::Flying;
    }

    virtual RCData getRCData() const override
    {
        //return what we received last time through setRCData
        return last_rcData_;
    }

    virtual GeoPoint getGpsLocation() const override
    {
        return AirSimPlaneFlightCommon::toGeoPoint(firmware_->offboardApi().getGeoPoint());
    }

    virtual float getCommandPeriod() const override
    {
        return 1.0f / 50; //50hz
    }

    virtual float getTakeoffZ() const override
    {
        // pick a number, 3 meters is probably safe
        // enough to get out of the backwash turbulence.  Negative due to NED coordinate system.
        return params_.takeoff.takeoff_z;
    }

    virtual float getDistanceAccuracy() const override
    {
        return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance traveled
    }

	// order here is not aetr, so be careful should be changed
    virtual void commandControls(float elevator, float aileron, float rudder, float tla) override
    {
        //Utils::log(Utils::stringf("commandControls %f, %f, %f, %f", elevator, aileron, rudder, tla));

        typedef plane_flight::GoalModeType GoalModeType;
        plane_flight::GoalMode mode(GoalModeType::Passthrough, GoalModeType::Passthrough, GoalModeType::Passthrough, GoalModeType::Passthrough);

        plane_flight::Axis4r goal(elevator, aileron, rudder, tla);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandRollPitchYawHold(float roll, float pitch, float yaw, float throttle) override
    {
        //Utils::log(Utils::stringf("commandRollPitchYawThrottle %f, %f, %f, %f", roll, pitch, yaw, throttle));

        typedef plane_flight::GoalModeType GoalModeType;
        plane_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::Passthrough);

        plane_flight::Axis4r goal(roll, pitch, yaw, throttle);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }
	
    virtual void commandAltitudeHold(float roll, float pitch, float yaw, float z) override
    {
        //Utils::log(Utils::stringf("commandRollPitchYawZ %f, %f, %f, %f", pitch, roll, z, yaw));

        typedef plane_flight::GoalModeType GoalModeType;
        plane_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::PositionWorld);

        plane_flight::Axis4r goal(roll, pitch, yaw, z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandAngleRates(float roll_rate, float pitch_rate, float yaw_rate, float tla) override
    {
        //Utils::log(Utils::stringf("commandRollPitchYawThrottle %f, %f, %f, %f", roll, pitch, yaw_rate, throttle));

        typedef plane_flight::GoalModeType GoalModeType;
        plane_flight::GoalMode mode(GoalModeType::AngleRate, GoalModeType::AngleRate, GoalModeType::AngleRate, GoalModeType::Passthrough);

        plane_flight::Axis4r goal(roll_rate, pitch_rate, yaw_rate, tla);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandVelocityHold(float vx, float vy, float vz) override
    {
        //Utils::log(Utils::stringf("commandVelocity %f, %f, %f, %f", vx, vy, vz, yaw_mode.yaw_or_rate));

        typedef plane_flight::GoalModeType GoalModeType;
        plane_flight::GoalMode mode(GoalModeType::VelocityWorld, GoalModeType::VelocityWorld, GoalModeType::VelocityWorld);

        plane_flight::Axis4r goal(vy, vx, vz);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandVelocityAltitudeHold(float vx, float vy, float z) override
    {
        //Utils::log(Utils::stringf("commandVelocityZ %f, %f, %f, %f", vx, vy, z, yaw_mode.yaw_or_rate));

        typedef plane_flight::GoalModeType GoalModeType;
        plane_flight::GoalMode mode(GoalModeType::VelocityWorld, GoalModeType::VelocityWorld, GoalModeType::PositionWorld);

        plane_flight::Axis4r goal(vx, vy, z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void setControllerGains(uint8_t controller_type, const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) override
    {
        plane_flight::GoalModeType controller_type_enum = static_cast<plane_flight::GoalModeType>(controller_type);

        vector<float> kp_axis4(4);
        vector<float> ki_axis4(4);
        vector<float> kd_axis4(4);

        switch(controller_type_enum) {
            // roll gain, pitch gain, yaw gain, and no gains in throttle / z axis
            case plane_flight::GoalModeType::AngleRate:
                kp_axis4 = {kp[0], kp[1], kp[2], 1.0};
                ki_axis4  ={ki[0], ki[1], ki[2], 0.0};
                kd_axis4 = {kd[0], kd[1], kd[2], 0.0};
                params_.angle_rate_pid.p.setValues(kp_axis4);
                params_.angle_rate_pid.i.setValues(ki_axis4);
                params_.angle_rate_pid.d.setValues(kd_axis4);
                params_.gains_changed = true;
                break;
            case plane_flight::GoalModeType::AngleLevel:
                kp_axis4 = {kp[0], kp[1], kp[2], 1.0};
                ki_axis4 = {ki[0], ki[1], ki[2], 0.0};
                kd_axis4 = {kd[0], kd[1], kd[2], 0.0};
                params_.angle_level_pid.p.setValues(kp_axis4);
                params_.angle_level_pid.i.setValues(ki_axis4);
                params_.angle_level_pid.d.setValues(kd_axis4);
                params_.gains_changed = true;
                break;
            case plane_flight::GoalModeType::VelocityWorld:
                kp_axis4 = {kp[1], kp[0], 0.0, kp[2]};
                ki_axis4 = {ki[1], ki[0], 0.0, ki[2]};
                kd_axis4 = {kd[1], kd[0], 0.0, kd[2]};
                params_.velocity_pid.p.setValues(kp_axis4);
                params_.velocity_pid.i.setValues(ki_axis4);
                params_.velocity_pid.d.setValues(kd_axis4);
                params_.gains_changed = true;
                break;
            case plane_flight::GoalModeType::PositionWorld:
                kp_axis4 = {kp[1], kp[0], 0.0, kp[2]};
                ki_axis4 = {ki[1], ki[0], 0.0, ki[2]};
                kd_axis4 = {kd[1], kd[0], 0.0, kd[2]};
                params_.position_pid.p.setValues(kp_axis4);
                params_.position_pid.i.setValues(ki_axis4);
                params_.position_pid.d.setValues(kd_axis4);
                params_.gains_changed = true;
                break;
            default:
                Utils::log("Unimplemented controller type");
                break;
        }
    }

    virtual void commandPositionHold(float x, float y, float z) override
    {
        //Utils::log(Utils::stringf("commandPosition %f, %f, %f, %f", x, y, z, yaw_mode.yaw_or_rate));

        typedef plane_flight::GoalModeType GoalModeType;
        plane_flight::GoalMode mode(GoalModeType::PositionWorld, GoalModeType::PositionWorld, GoalModeType::PositionWorld);

        plane_flight::Axis4r goal(x, y, z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual const FixedWingApiParams& GetFixedWingApiParams() const override
    {
        return safety_params_;
    }

    //*** End: FixedWingApiBase implementation ***//

private:
    //convert pitch, roll, yaw from -1 to 1 to PWM
    static uint16_t angleToPwm(float angle)
    {
        return static_cast<uint16_t>(angle * 500.0f + 1500.0f);
    }
    static uint16_t thrustToPwm(float thrust)
    {
        return static_cast<uint16_t>((thrust < 0 ? 0 : thrust) * 1000.0f + 1000.0f);
    }
    static uint16_t switchTopwm(float switchVal, uint maxSwitchVal = 1)
    {
        return static_cast<uint16_t>(1000.0f * switchVal / maxSwitchVal + 1000.0f);
    }

    void readSettings(const AirSimSettings::VehicleSetting& vehicle_setting)
    {
        params_.default_vehicle_state = plane_flight::VehicleState::fromString(
            vehicle_setting.default_vehicle_state == "" ? "Armed" : vehicle_setting.default_vehicle_state);

        remote_control_id_ = vehicle_setting.rc.remote_control_id;
        params_.rc.allow_api_when_disconnected = vehicle_setting.rc.allow_api_when_disconnected;
        params_.rc.allow_api_always = vehicle_setting.allow_api_always;
    }

private:
    const FixedWingParams* vehicle_params_;

    int remote_control_id_ = 0;
    plane_flight::Params params_;

    unique_ptr<AirSimPlaneFlightBoard> board_;
    unique_ptr<AirSimPlaneFlightCommLink> comm_link_;
    unique_ptr<AirSimPlaneFlightEstimator> estimator_;
    unique_ptr<plane_flight::IFirmware> firmware_;

    FixedWingApiParams safety_params_;

    RCData last_rcData_;

    real_T elevator_deflection_;
    real_T aileron_deflection_;
    real_T rudder_deflection_;
    real_T tla_deflection_;
};

}} //namespace
#endif
