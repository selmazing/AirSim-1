// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_ArduPlane_hpp
#define msr_airlib_vehicles_ArduPlane_hpp

#include "vehicles/fixedwing/firmwares/ardupilot/ArduPlaneApi.hpp"
#include "vehicles/fixedwing/FixedWingParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"


namespace msr { namespace airlib {

class ArduPlaneParams : public FixedWingParams {
public:
    ArduPlaneParams(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
        : sensor_factory_(sensor_factory)
    {
        connection_info_ = getConnectionInfo(vehicle_setting);
    }

    virtual ~ArduPlaneParams() = default;

    virtual std::unique_ptr<FixedWingApiBase> createFixedWingApi() override
    {
        return std::unique_ptr<FixedWingApiBase>(new ArduPlaneApi(this, connection_info_));
    }

protected:
    virtual void setupParams() override
    {
        auto& params = getParams();

        // Use connection_info_.model for the model name, see Px4MultiRotorParams for example

        /* Only Generic for now
        setupFrameGenericAircraft(params); */

    	// Using Skywalker X8 instead
        setupFrameSkywalkerX8(params);
    }

    virtual const SensorFactory* getSensorFactory() const override
    {
        return sensor_factory_.get();
    }

    static const AirSimSettings::MavLinkConnectionInfo& getConnectionInfo(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting)
    {
        return vehicle_setting.connection_info;
    }

private:
    AirSimSettings::MavLinkConnectionInfo connection_info_;
    std::shared_ptr<const SensorFactory> sensor_factory_;
};

}} //namespace
#endif
