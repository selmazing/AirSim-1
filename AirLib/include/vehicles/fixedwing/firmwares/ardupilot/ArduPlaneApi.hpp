// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_ArduPlaneDroneController_hpp
#define msr_airlib_ArduPlaneDroneController_hpp

#include "vehicles/fixedwing/api/FixedWingApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/fixedwing/FixedWingParams.hpp"
#include "common/Common.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/AirSimSettings.hpp"

// Sensors
#include "sensors/imu/ImuBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/lidar/LidarBase.hpp"

#include "UdpSocket.hpp"

#include <sstream>
#include <iostream>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

namespace msr { namespace airlib {

class ArduPlaneApi : public FixedWingApiBase {

public:
    ArduPlaneApi(const FixedWingParams* vehicle_params, const AirSimSettings::MavLinkConnectionInfo& connection_info)
        : ip_(connection_info.udp_address), vehicle_params_(vehicle_params)
    {
        connection_info_ = connection_info;
        sensors_ = &getSensors();

        connect(); // Should we try catching exceptions here?
    }

    ~ArduPlaneApi()
    {
        closeConnections();
    }


public:
    virtual void resetImplementation() override
    {
        FixedWingApiBase::resetImplementation();

        // Reset state
    }

    // Update sensor data & send to Ardupilot
    virtual void update() override
    {
        FixedWingApiBase::update();
        sendSensors();
        recvControlDeflection();
    }

    // TODO:VehicleApiBase implementation
    virtual bool isApiControlEnabled() const override
    {
        Utils::log("Not Implemented: isApiControlEnabled", Utils::kLogLevelInfo);
        return false;
    }
    virtual void enableApiControl(bool is_enabled) override
    {
        Utils::log("Not Implemented: enableApiControl", Utils::kLogLevelInfo);
        unused(is_enabled);
    }
    virtual bool armDisarm(bool arm) override
    {
        Utils::log("Not Implemented: armDisarm", Utils::kLogLevelInfo);
        unused(arm);
        return false;
    }
    virtual GeoPoint getHomeGeoPoint() const override
    {
        Utils::log("Not Implemented: getHomeGeoPoint", Utils::kLogLevelInfo);
        return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
    }
    virtual void getStatusMessages(std::vector<std::string>& messages) override
    {
        unused(messages);
    }

    virtual const SensorCollection& getSensors() const override
    {
        return vehicle_params_->getSensors();
    }

public:

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
        Utils::log("Not Implemented: setSimulatedGroundTruth", Utils::kLogLevelInfo);
        unused(kinematics);
        unused(environment);
    }

    virtual bool setRCData(const RCData& rc_data) override
    {
        last_rcData_ = rc_data;
        is_rc_connected_ = true;

        return true;
    }

protected:
    virtual Kinematics::State getKinematicsEstimated() const override
    {
        Utils::log("Not Implemented: getKinematicsEstimated", Utils::kLogLevelInfo);
        Kinematics::State state;
        return state;
    }

    virtual Vector3r getPosition() const override
    {
        Utils::log("Not Implemented: getPosition", Utils::kLogLevelInfo);
        return Vector3r(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
    }

    virtual Vector3r getVelocity() const override
    {
        Utils::log("Not Implemented: getVelocity", Utils::kLogLevelInfo);
        return Vector3r(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
    }

    virtual Quaternionr getOrientation() const override
    {
        Utils::log("Not Implemented: getOrientation", Utils::kLogLevelInfo);
        return Quaternionr(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
    }

    virtual LandedState getLandedState() const override
    {
        Utils::log("Not Implemented: getLandedState", Utils::kLogLevelInfo);
        return LandedState::Landed;
    }

    virtual RCData getRCData() const override
    {
        //return what we received last time through setRCData
        return last_rcData_;
    }

    virtual GeoPoint getGpsLocation() const override
    {
        Utils::log("Not Implemented: getGpsLocation", Utils::kLogLevelInfo);
        return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
    }

    virtual float getCommandPeriod() const override
    {
        return 1.0f / 50; //50hz
    }

    virtual float getTakeoffZ() const override
    {
        // pick a number (Height), 3 meters is probably safe
        // enough to get out of the backwash turbulence.  Negative due to NED coordinate system.
        // return params_.takeoff.takeoff_z;
        return 3.0;
    }

    virtual float getDistanceAccuracy() const override
    {
        return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance traveled
    }

    virtual void setControllerGains(uint8_t controllerType, const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) override
    {
        unused(controllerType);
        unused(kp);
        unused(ki);
        unused(kd);
        Utils::log("Not Implemented: setControllerGains", Utils::kLogLevelInfo);
    }

    virtual void commandControls(float elevator, float aileron, float rudder, float tla) override
    {
        unused(elevator);
        unused(aileron);
        unused(rudder);
        unused(tla);
        Utils::log("Not Implemented: commandControls", Utils::kLogLevelInfo);
    }

    virtual void commandRollPitchYawHold(float roll, float pitch, float yaw, float tla) override
    {
        unused(roll);
        unused(pitch);
        unused(yaw);
        unused(tla);
        Utils::log("Not Implemented: commandAttititudeHold", Utils::kLogLevelInfo);
    }

    virtual void commandAltitudeHold(float roll, float pitch, float yaw, float z) override
    {
        unused(roll);
        unused(pitch);
        unused(yaw);
        unused(z);
        Utils::log("Not Implemented: commandAltitudeHold", Utils::kLogLevelInfo);
    }

    virtual void commandAngleRates(float roll_rate, float pitch_rate, float yaw_rate, float tla) override
    {
        unused(roll_rate);
        unused(pitch_rate);
        unused(yaw_rate);
        unused(tla);
        Utils::log("Not Implemented: commandAngleRates", Utils::kLogLevelInfo);
    }

    virtual void commandVelocityHold(float vx, float vy, float vz) override
    {
        unused(vx);
        unused(vy);
        unused(vz);
        Utils::log("Not Implemented: commandVelocityHold", Utils::kLogLevelInfo);
    }

    virtual void commandVelocityAltitudeHold(float vx, float vy, float vz) override
    {
        unused(vx);
        unused(vy);
        unused(vz);
        Utils::log("Not Implemented: commandVelocityZ", Utils::kLogLevelInfo);
    }

    virtual void commandPositionHold(float x, float y, float z) override
    {
        unused(x);
        unused(y);
        unused(z);
        Utils::log("Not Implemented: commandPosition", Utils::kLogLevelInfo);
    }

    virtual const FixedWingApiParams& GetFixedWingApiParams() const override
    {
        return safety_params_;
    }

    //*** End: MultirotorApiBase implementation ***//

protected:
    void closeConnections()
    {
        if (udpSocket_ != nullptr)
            udpSocket_->close();
    }

    void connect()
    {
        port_ = static_cast<uint16_t>(connection_info_.udp_port);

        closeConnections();

        if (ip_ == "") {
            throw std::invalid_argument("UdpIp setting is invalid.");
        }

        if (port_ == 0) {
            throw std::invalid_argument("UdpPort setting has an invalid value.");
        }

        Utils::log(Utils::stringf("Using UDP port %d, local IP %s, remote IP %s for sending sensor data", port_, connection_info_.local_host_ip.c_str(), ip_.c_str()), Utils::kLogLevelInfo);
        Utils::log(Utils::stringf("Using UDP port %d for receiving rotor power", connection_info_.control_port, connection_info_.local_host_ip.c_str(), ip_.c_str()), Utils::kLogLevelInfo);

        udpSocket_ = std::make_shared<mavlinkcom::UdpSocket>();
        udpSocket_->bind(connection_info_.local_host_ip, connection_info_.control_port);
    }

private:
    /* virtual void normalizeRotorControls()
    {
        // change 1000-2000 to 0-1.
        for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
            rotor_controls_[i] = (rotor_controls_[i] - 1000.0f) / 1000.0f;
        }
    } */

    void sendSensors()
    {
        if (sensors_ == nullptr)
            return;

        const auto& gps_output = getGpsData("");
        const auto& imu_output = getImuData("");

        std::ostringstream oss;

        // Send RC channels to Ardupilot if present
        if (is_rc_connected_ && last_rcData_.is_valid) {
            oss << ","
                   "\"rc\": {"
                   "\"channels\": ["
                << (last_rcData_.roll + 1) * 0.5f << ","
                << (last_rcData_.yaw + 1) * 0.5f << ","
                << (last_rcData_.throttle + 1) * 0.5f << ","
                << (-last_rcData_.pitch + 1) * 0.5f << ","
                << static_cast<float>(last_rcData_.getSwitch(0)) << ","
                << static_cast<float>(last_rcData_.getSwitch(1)) << ","
                << static_cast<float>(last_rcData_.getSwitch(2)) << ","
                << static_cast<float>(last_rcData_.getSwitch(3))
                << "]}";
        }

        const uint count_lidars = getSensors().size(SensorBase::SensorType::Lidar);
    	//Do we want the aircraft to have LIDAR?
        // Since it's possible that we don't want to send the lidar data to Ardupilot but still have the lidar (maybe as a ROS topic)
        if (count_lidars != 0) {
            const auto& lidar_output = getLidarData("");
            oss << ","
                   "\"lidar\": {"
                   "\"point_cloud\": [";

            std::copy(lidar_output.point_cloud.begin(), lidar_output.point_cloud.end(), std::ostream_iterator<real_T>(oss, ","));
            oss << "]}";
        }

        float yaw;
        float pitch;
        float roll;
        VectorMath::toEulerianAngle(imu_output.orientation, pitch, roll, yaw);

        char buf[65000]; // Is this the port?

        // TODO: Split the following sensor packet formation into different parts for individual sensors

        // UDP packets have a maximum size limit of 65kB
    	// We will need a pitot tube as well -> add to sensors maybe speed barometer
        int ret = snprintf(buf, sizeof(buf),
                           "{"
                           "\"timestamp\": %" PRIu64 ","
                           "\"imu\": {"
                           "\"angular_velocity\": [%.12f, %.12f, %.12f],"
                           "\"linear_acceleration\": [%.12f, %.12f, %.12f]"
                           "},"
                           "\"gps\": {"
                           "\"lat\": %.7f,"
                           "\"lon\": %.7f,"
                           "\"alt\": %.3f"
                           "},"
                           "\"velocity\": {"
                           "\"world_linear_velocity\": [%.12f, %.12f, %.12f]"
                           "},"
                           "\"pose\": {"
                           "\"roll\": %.12f,"
                           "\"pitch\": %.12f,"
                           "\"yaw\": %.12f"
                           "}"
                           "%s"
                           "}\n",
                           static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E3),	
                           imu_output.angular_velocity[0],
                           imu_output.angular_velocity[1],
                           imu_output.angular_velocity[2],
                           imu_output.linear_acceleration[0],
                           imu_output.linear_acceleration[1],
                           imu_output.linear_acceleration[2],
                           gps_output.gnss.geo_point.latitude,
                           gps_output.gnss.geo_point.longitude,
                           gps_output.gnss.geo_point.altitude,
                           gps_output.gnss.velocity[0],
                           gps_output.gnss.velocity[1],
                           gps_output.gnss.velocity[2],
                           roll, pitch, yaw,
                           oss.str().c_str());

        if (ret < 0) {
            Utils::log("Encoding error while forming sensor message", Utils::kLogLevelInfo);
            return;
        }
        else if (static_cast<uint>(ret) >= sizeof(buf)) {
            Utils::log(Utils::stringf("Sensor message truncated, lost %d bytes", ret - sizeof(buf)), Utils::kLogLevelInfo);
        }

        // Send data
        if (udpSocket_ != nullptr) {
            udpSocket_->sendto(buf, strlen(buf), ip_, port_);
        }
    }

	void recvControlDeflection()
    {
	    //Receive control surface data
        PlaneControlMessage pkt;
        int recv_ret = udpSocket_->recv(&pkt, sizeof(pkt), 100);
	    while (recv_ret != sizeof(pkt))
	    {
		    if (recv_ret != sizeof(pkt))
		    {
                Utils::log(Utils::stringf("Error while receiving aircraft control data - ErrorNo: %d", recv_ret), Utils::kLogLevelInfo);
		    }   else
		    {
                Utils::log(Utils::stringf("Received %d bytes instead of %zu bytes", recv_ret, sizeof(pkt)), Utils::kLogLevelInfo);
		    }

            recv_ret = udpSocket_->recv(&pkt, sizeof(pkt), 100);
	    }


        elevator_deflection_ = pkt.elevator_deflection;
        aileron_deflection_ = pkt.aileron_deflection;
        tla_deflection_ = pkt.tla_deflection;
        rudder_deflection_ = pkt.rudder_deflection;
        /* Utils::log(Utils::stringf("Received TLA: %f", tla_deflection_), Utils::kLogLevelInfo);
        Utils::log(Utils::stringf("Received elevator: %f", elevator_deflection_), Utils::kLogLevelInfo);
        Utils::log(Utils::stringf("Received aileron: %f", aileron_deflection_), Utils::kLogLevelInfo);
        Utils::log(Utils::stringf("Received rudder: %f", rudder_deflection_), Utils::kLogLevelInfo); */
    }

private:
	// This is inheited from Multirotor for ArduCopter, think this needs to reflect controls in SITL ArduPlane
    static const int kArduPilotControlCount = 11;
	
    struct PlaneControlMessage
    {
        uint16 aileron_deflection;
        uint16 elevator_deflection;
        uint16 tla_deflection;
        uint16 rudder_deflection;
    };

    std::shared_ptr<mavlinkcom::UdpSocket> udpSocket_;

    AirSimSettings::MavLinkConnectionInfo connection_info_;
    uint16_t port_;
    const std::string& ip_;
    const SensorCollection* sensors_;
    const FixedWingParams* vehicle_params_;

    FixedWingApiParams safety_params_;

    RCData last_rcData_;
    bool is_rc_connected_;

    real_T elevator_deflection_;
    real_T aileron_deflection_;
    real_T rudder_deflection_;
    real_T tla_deflection_;
};

}} //namespace
#endif
